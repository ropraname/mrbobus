"""No ROS dependency: deterministic safety, CAN codec, kinematics and odometry.

Only tick() emits periodic CAN. Every external command is validated here.
Time arguments MUST be monotonic; source timestamps are checked by adapters.
"""
import math
import struct
from dataclasses import dataclass

SIGNS = {0: 1, 1: -1, 2: -1, 3: 1}
NAMES = {0: 'rear_right_wheel', 1: 'rear_left_wheel',
         2: 'front_left_wheel', 3: 'front_right_wheel'}
CPR = 72
RADIUS = .045
TRACK = .29174
WHEELBASE = .215562551226
ACCEL_RPS = .2
COMMAND_TIMEOUT = .3
FEEDBACK_TIMEOUT = .5

def frame(node, command, payload=b'', rtr=False):
    if node not in SIGNS or not 0 <= command <= 31 or len(payload) > 8:
        raise ValueError('CAN frame')
    return struct.pack('=IB3x8s', (node << 5) | command | (0x40000000 if rtr else 0),
                       8 if rtr else len(payload), payload.ljust(8, b'\0'))

def unpack(raw):
    if len(raw) != 16:
        return None
    ident, size, payload = struct.unpack('=IB3x8s', raw)
    if ident & 0xE0000000 or size != 8 or ident >> 5 not in SIGNS:
        return None
    return ident >> 5, ident & 31, payload

def wheels(linear, angular):
    if not all(math.isfinite(v) for v in (linear, angular)):
        raise ValueError('Non-finite command')
    left = (linear - angular*TRACK/2)/(2*math.pi*RADIUS)
    right = (linear + angular*TRACK/2)/(2*math.pi*RADIUS)
    if not all(math.isfinite(v) and abs(v) < 3.4e38 for v in (left, right)):
        raise ValueError('Command cannot be represented on CAN')
    return {n: SIGNS[n]*(left if n in (1, 2) else right) for n in SIGNS}

def signed_delta(new, old):
    return ((new-old+2**31) % 2**32)-2**31

@dataclass
class Axis:
    heartbeat: float = -1e9
    count_time: float = -1e9
    state: int = 0
    error: int = 0
    count: object = None
    # Signed physical-forward revolutions, preserved across rejected baselines.
    turns: float = 0.
    motor_error: int = 0
    encoder_error: int = 0
    controller_error: int = 0
    voltage: object = None
    current: object = None

class Base:
    def __init__(self, send, clock, hardware_enabled=False, session_limit=0.):
        self.send = send
        self.clock = clock
        self.hardware_enabled = hardware_enabled
        self.session_limit = session_limit
        self.axes = {n: Axis() for n in SIGNS}
        self.state = 'IDLE'
        self.reason = 'Startup: disarmed'
        self.owner = None
        self.last_seq = -1
        self.command_time = -1e9
        self.target = {n: 0. for n in SIGNS}
        self.output = dict(self.target)
        self.pending = []
        self.started = 0.
        self.last_tick = clock()
        self.stop_started = 0.
        self.stop_until = -1.
        self.last_query = -1.
        self.last_diag = -1.
        self.odom_ref = None
        self.x = self.y = self.yaw = self.v = self.w = 0.
        self.odom_time = clock()
        self.odom_valid = False

    def emit(self, n, cmd, payload=b'', rtr=False):
        self.send(frame(n, cmd, payload, rtr))

    def fresh(self, now):
        return all(now-a.heartbeat <= FEEDBACK_TIMEOUT and now-a.count_time <= FEEDBACK_TIMEOUT
                   for a in self.axes.values())

    def receive(self, raw, now):
        decoded = unpack(raw)
        if decoded is None:
            return
        n, cmd, payload = decoded
        a = self.axes[n]
        if cmd == 1:
            err, state = struct.unpack_from('<IB', payload)
            a.heartbeat, a.error, a.state = now, err, state
            if self.state in ('RUNNING', 'STOPPING') and (err or state != 8):
                self.fault(f'Axis {n}: error={err}, state={state}', now)
            elif self.state == 'ARMING' and (err not in (0, 2048) or state not in (1, 8)):
                self.fault(f'Axis {n} failed to arm', now)
            elif self.state == 'IDLE' and state != 1 and now > self.stop_until:
                self.fault(f'Unexpected active axis {n}', now)
            elif self.state == 'IDLE' and err not in (0, 2048) and now > self.stop_until:
                self.fault(f'Axis {n} fault {err}', now)
        elif cmd == 10:
            count, within = struct.unpack('<ii', payload)
            gap = now-a.count_time
            delta = signed_delta(count, a.count) if a.count is not None else 0
            # Rebase after silence/disarm, and reject physically implausible jumps.
            # Reboot during motion is also caught by loss of state 8 in heartbeat.
            jump = abs(delta) > max(4., 2.5*CPR*max(0., min(gap, .5)))
            if a.count is not None and gap <= .5 and not jump:
                # Idle is also the reboot state: do not interpret a reset counter
                # as vehicle travel. Passive hand-rotation remains visible in count.
                if self.state in ('RUNNING', 'STOPPING'):
                    a.turns += delta/CPR*SIGNS[n]
            else:
                self.odom_ref = None
                self.odom_valid = False
                if self.state in ('ARMING', 'RUNNING', 'STOPPING'):
                    self.fault(f'Axis {n}: count discontinuity/stale data', now)
            a.count, a.count_time = count, now
        elif cmd in (3, 4, 0x1D):
            field = {3: 'motor_error', 4: 'encoder_error', 0x1D: 'controller_error'}[cmd]
            setattr(a, field, int.from_bytes(payload, 'little'))
        elif cmd == 0x17:
            volts, amps = struct.unpack('<ff', payload)
            if math.isfinite(volts) and math.isfinite(amps):
                a.voltage, a.current = volts, amps

    def arm(self, owner, now):
        if not self.hardware_enabled:
            return False, 'Motion interlock disabled: commissioning required'
        if self.state != 'IDLE' or not self.fresh(now):
            return False, 'Need Idle and fresh feedback from all axes'
        if any(a.state != 1 or a.error not in (0, 2048) for a in self.axes.values()):
            return False, 'Axis not ready'
        self.owner, self.last_seq = owner, -1
        self.command_time = now
        self.started = now
        self.state, self.reason = 'ARMING', 'Explicit arm requested'
        self.target = {n: 0. for n in SIGNS}
        self.output = dict(self.target)
        self.odom_ref = None
        # Spread setup over ticks: do not overflow the MCP2515 TX queue.
        self.pending = [(n, cmd, data) for cmd, data in [
            (0x0D, struct.pack('<ff', 0, 0)),
            (0x0F, struct.pack('<ff', 2, 15)),
            (0x0B, struct.pack('<ii', 2, 1)),
            (0x18, b''), (7, struct.pack('<I', 8))] for n in SIGNS]
        return True, 'Arming at zero speed'

    def command(self, owner, seq, linear, angular, now):
        if owner != self.owner or self.state not in ('ARMING', 'RUNNING'):
            return False
        if type(seq) is not int or seq <= self.last_seq:
            return False
        try:
            target = wheels(linear, angular)
        except (ValueError, TypeError, OverflowError):
            self.fault('Invalid velocity command', now)
            return False
        self.last_seq, self.command_time, self.target = seq, now, target
        return True

    def stop(self, reason, now, emergency=False):
        if emergency:
            self.fault(reason, now)
        elif self.state in ('RUNNING', 'ARMING'):
            if self.state == 'ARMING':
                self.disarm(reason, now)
            else:
                self.state, self.reason = 'STOPPING', reason
                self.stop_started = now
                self.target = {n: 0. for n in SIGNS}

    def disarm(self, reason, now):
        self.state, self.reason, self.owner = 'IDLE', reason, None
        self.pending.clear()
        self.output = {n: 0. for n in SIGNS}
        self.stop_until = now+.4
        self.odom_ref = None

    def fault(self, reason, now):
        if self.state != 'FAULT':
            self.reason = reason
            self.stop_until = now+.4
        self.state, self.owner = 'FAULT', None
        self.pending.clear()
        self.output = {n: 0. for n in SIGNS}
        self.odom_ref = None
        self.odom_valid = False

    def reset(self, now):
        if self.state != 'FAULT':
            return False, 'No latched fault'
        if any(now-a.heartbeat > .5 or a.state != 1 for a in self.axes.values()):
            return False, 'Need fresh Idle from every axis'
        # Explicit reset clears errors but NEVER arms; require new command lease.
        for n in SIGNS:
            self.emit(n, 0x18)
        self.disarm('Fault acknowledged; remain disarmed', now)
        return True, self.reason

    def tick(self, now):
        dt = now-self.last_tick
        self.last_tick = now
        if self.state == 'IDLE' and now > self.stop_until:
            if any(now-a.heartbeat < .5 and (a.state != 1 or a.error not in (0,2048)) for a in self.axes.values()):
                self.fault('Idle not confirmed / unresolved axis error', now)
        if self.state in ('ARMING', 'RUNNING', 'STOPPING'):
            if dt > .2 or dt < 0:
                self.fault('Control loop deadline missed', now)
            elif not self.fresh(now):
                self.fault('Stale heartbeat/raw counts', now)
        if self.state == 'RUNNING':
            if now-self.command_time > COMMAND_TIMEOUT:
                self.stop(f'Command expired after {now-self.command_time:.2f} s', now)
            elif self.session_limit > 0 and now-self.started >= self.session_limit:
                self.stop('Commissioning session time limit', now)
        if self.state == 'ARMING':
            for _ in range(min(4, len(self.pending))):
                n, cmd, data = self.pending.pop(0)
                self.emit(n, cmd, data)
            if not self.pending and all(a.state == 8 and a.error == 0 for a in self.axes.values()):
                if self.last_seq < 0:
                    # Arming configures all four axes at zero speed. Start the
                    # command deadline when setup finishes if the browser has
                    # not received the lease and sent its first command yet.
                    self.command_time = now
                self.state, self.reason = 'RUNNING', 'Deadman control active'
            elif now-self.started > 1.:
                self.fault('Closed-loop entry timeout', now)
        elif self.state in ('RUNNING', 'STOPPING'):
            # One common ramp factor preserves left/right curvature.
            diffs = {n: self.target[n]-self.output[n] for n in SIGNS}
            largest = max(abs(d) for d in diffs.values())
            scale = min(1., ACCEL_RPS*max(0., min(dt, .1))/largest) if largest else 1.
            for n in SIGNS:
                self.output[n] += diffs[n]*scale
                self.emit(n, 0x0D, struct.pack('<ff', self.output[n], 0))
            if self.state == 'STOPPING' and (max(abs(v) for v in self.output.values()) < 1e-6 or now-self.stop_started > 1.1):
                self.disarm(self.reason, now)
        # Fault mode emits ONLY zero+Idle for a short burst, then no requests.
        # In particular diagnostics must not keep the hardware watchdog fed.
        if self.state in ('FAULT', 'IDLE') and now <= self.stop_until:
            for n in SIGNS:
                self.emit(n, 7, struct.pack('<I', 1))
        elif self.state != 'FAULT' and now-self.last_query >= .049:
            for n in SIGNS:
                self.emit(n, 10, rtr=True)
            self.last_query = now
        if self.state == 'IDLE' and now > self.stop_until and now-self.last_diag >= 1.:
            # One diagnostic type each cycle; avoid burst and monitor load.
            cmd = (3, 4, 0x1D, 0x17)[int(now) % 4]
            for n in SIGNS:
                self.emit(n, cmd, rtr=True)
            self.last_diag = now
        self.update_odometry(now)

    def update_odometry(self, now):
        if not self.fresh(now) or self.state == 'FAULT':
            self.odom_ref = None
            self.odom_valid = False
            self.v = self.w = 0.
            return
        current = {n: a.turns for n, a in self.axes.items()}
        if self.odom_ref is not None:
            d = {n: (current[n]-self.odom_ref[n])*2*math.pi*RADIUS for n in SIGNS}
            dl, dr = (d[1]+d[2])/2, (d[0]+d[3])/2
            ds, dyaw = (dl+dr)/2, (dr-dl)/TRACK
            self.x += ds*math.cos(self.yaw+dyaw/2)
            self.y += ds*math.sin(self.yaw+dyaw/2)
            self.yaw = math.atan2(math.sin(self.yaw+dyaw), math.cos(self.yaw+dyaw))
            dt = now-self.odom_time
            self.v, self.w = (ds/dt, dyaw/dt) if dt > 0 else (0.,0.)
        self.odom_ref, self.odom_time, self.odom_valid = current, now, True

    def status(self, now):
        return {'state': self.state, 'reason': self.reason, 'motion_enabled': self.hardware_enabled,
                'session_limit_s': self.session_limit, 'odom_valid': self.odom_valid,
                'odom': [self.x,self.y,self.yaw], 'owner_active': self.owner is not None,
                'axes': {n: {'name': NAMES[n], 'state': a.state, 'error': a.error,
                    'heartbeat_age': round(now-a.heartbeat,3), 'count_age': round(now-a.count_time,3),
                    'count': a.count, 'turns': a.turns, 'target_rpm': self.output[n]*60,
                    'motor_error': a.motor_error, 'encoder_error': a.encoder_error,
                    'controller_error': a.controller_error, 'voltage': a.voltage, 'current': a.current}
                         for n,a in self.axes.items()}}
