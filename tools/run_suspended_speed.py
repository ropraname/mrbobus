#!/usr/bin/env python3
"""Exclusive, temporary CAN 0.5.6 speed sweep. NOT for floor operation.

Stop robot-base/mrbobus-control first. Requires an explicit USB settings snapshot;
restores limits in RAM, never saves flash or changes P/I. STOP is never cleared.
"""
import argparse
import errno
import fcntl
import json
import math
from pathlib import Path
import signal
import socket
import struct
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--can', choices=['can0', 'vcan0'], default='vcan0')
    p.add_argument('--suspended-wheels', action='store_true', required=True)
    p.add_argument('--kmh', type=float, default=10.)
    p.add_argument('--baseline', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    if not math.isfinite(args.kmh) or not 1 <= args.kmh <= 10:
        p.error('Only 1–10 km/h is supported')
    baseline = json.loads(Path(args.baseline).read_text())
    assert set(baseline) == {'0', '1', '2', '3'}
    for a in baseline.values():
        assert a['watchdog'] and 0 < a['watchdog_timeout'] <= 2
        assert a['control_mode'] == 2 and a['input_mode'] == 1
        assert a['velocity_limit'] == 3 and a['current_limit'] == 20
    physical = args.can == 'can0'
    stop_path = Path('/run/mrbobus-stop/enabled' if physical else '/tmp/speed-vcan-stop')
    lock_path = '/run/robot-base/robot-base-can0.lock' if physical else '/tmp/robot-base-vcan0.lock'
    lock = open(lock_path, 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.bind((args.can,)); bus.setblocking(False)
    axes = {}; volts = {}; rows = []; changed = False; armed = False
    target = args.kmh / 3.6 / (math.pi * .09)
    speed = 0.; phase = 'prepare'; last_query = 0.
    signs = [1, -1, -1, 1]
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    def send(n, cmd, data=b'', rtr=False):
        ident = (n << 5) | cmd | (socket.CAN_RTR_FLAG if rtr else 0)
        packet = struct.pack('=IB3x8s', ident, 8 if rtr else len(data), data.ljust(8, b'\0'))
        deadline = time.monotonic()+.1
        while True:
            try:
                bus.send(packet); break
            except OSError as e:
                if e.errno not in (errno.ENOBUFS, errno.EAGAIN) or time.monotonic() >= deadline: raise
                time.sleep(.002)
        time.sleep(.0015)
    def velocity(v):
        for n in range(4): send(n, 13, struct.pack('<ff', signs[n]*v, 0.))
    def read():
        while True:
            try: raw = bus.recv(16)
            except BlockingIOError: break
            ident, size, data = struct.unpack('=IB3x8s', raw)
            if ident & 0xe0000000 or size != 8: continue
            n, cmd = ident >> 5, ident & 31
            if n not in range(4): continue
            a = axes.setdefault(n, {}); now = time.monotonic()
            if cmd == 1: a.update(error=struct.unpack_from('<I', data)[0], state=data[4], heartbeat=now)
            elif cmd == 9:
                pos, vel = struct.unpack('<ff', data)
                a.update(position=pos, velocity=vel, feedback=now)
            elif cmd == 20:
                demand, measured = struct.unpack('<ff', data)
                a.update(iq_target=demand, iq=measured)
            elif cmd == 23 and n in (0, 2): volts[n] = (struct.unpack('<ff', data)[0], now)
    def gate():
        if stop_path.read_text().strip() != '1': raise RuntimeError('STOP')
        if physical:
            radio = json.loads(Path('/run/mrbobus-stop/radio.json').read_text())
            if time.monotonic()-radio['at'] > .5 or radio['mode'] != 'auto' or radio['held']:
                raise RuntimeError('Radio takeover/STOP')
    def tick(check=True):
        nonlocal last_query
        gate(); velocity(speed)
        now = time.monotonic()
        if now-last_query >= .1:
            for n in range(4): send(n, 20, rtr=True)
            for n in (0, 2): send(n, 23, rtr=True)
            last_query = now
        read()
        if check:
            for n in range(4):
                a = axes.get(n, {})
                if now-a.get('heartbeat', 0) > .4 or now-a.get('feedback', 0) > .3:
                    raise RuntimeError('Stale axis '+str(n))
                if a.get('state') != 8 or a.get('error') != 0:
                    raise RuntimeError('Axis fault '+str(axes))
                v = a['velocity']
                if not math.isfinite(v) or abs(v) > min(12., max(1., speed+1.5)):
                    raise RuntimeError('Speed envelope '+str(axes))
            if len(volts) != 2 or any(now-at > .5 or not 30 <= v <= 43 for v, at in volts.values()):
                raise RuntimeError('Bus voltage envelope '+str(volts))
        rows.append({'t': now, 'phase': phase, 'command_rps': speed,
                     'axes': {n: dict(a) for n, a in axes.items()}, 'volts': dict(volts)})
        time.sleep(.02)
    def hold(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end: tick()
    def ramp(to):
        nonlocal speed
        previous = time.monotonic()
        while abs(speed-to) > 1e-6:
            now = time.monotonic(); step = min(.025, .5*(now-previous)); previous = now
            speed += max(-step, min(step, to-speed)); tick()
    def interrupted(*_): raise RuntimeError('Signal: stop trial')
    signal.signal(signal.SIGTERM, interrupted); signal.signal(signal.SIGINT, interrupted)
    result = 'failed'
    try:
        gate(); end = time.monotonic()+.6
        while time.monotonic() < end: read(); time.sleep(.01)
        if len(axes) != 4 or any(a.get('state') != 1 or a.get('error') not in (0, 2048) for a in axes.values()):
            raise RuntimeError('Expected all Idle, only watchdog timeout allowed')
        # Record rollback settings before the first write.
        output.with_suffix('.baseline.json').write_text(json.dumps(baseline, indent=2))
        changed = True
        for n in range(4):
            send(n, 15, struct.pack('<ff', 11.5, 5.))
            send(n, 11, struct.pack('<ii', 2, 1))
            send(n, 24, b'\0'*8)
        velocity(0.)
        armed = True
        for n in range(4): send(n, 7, struct.pack('<I', 8))
        end = time.monotonic()+.4
        while time.monotonic() < end:
            tick(check=False)
            if any(a.get('error', 0) not in (0, 2048) or abs(a.get('velocity', 0)) > 1 for a in axes.values()):
                raise RuntimeError('Startup fault/speed')
        phase = 'zero'; hold(1.)
        for value in sorted(set([min(target, x) for x in (1., 3., 5., 7., target)])):
            phase = 'ramp'; ramp(value)
            phase = f'hold_{value:.4f}'; print(phase, flush=True); hold(2.)
        phase = 'decelerate'; ramp(0.)
        phase = 'zero_final'; hold(2.)
        result = 'passed'
    finally:
        cleanup_errors = []
        def cleanup_send(n, cmd, data):
            try: send(n, cmd, data)
            except OSError as e: cleanup_errors.append(str(e))
        if armed:
            for n in range(4): cleanup_send(n, 13, struct.pack('<ff', 0., 0.))
            for n in range(4): cleanup_send(n, 7, struct.pack('<I', 1))
            end = time.monotonic()+.4
            while time.monotonic() < end: read(); time.sleep(.01)
        if changed:
            for n in range(4):
                a = baseline[str(n)]
                cleanup_send(n, 15, struct.pack('<ff', a['velocity_limit'], a['current_limit']))
        output.write_text(json.dumps({'result': result, 'samples': rows, 'final_axes': axes,
                                     'restore_sent': changed and not cleanup_errors,
                                     'cleanup_errors': cleanup_errors}, indent=2))
        bus.close(); lock.close()
        if cleanup_errors: raise RuntimeError('Cleanup needs USB recovery: '+str(cleanup_errors))
    if armed and any(a.get('state') != 1 for a in axes.values()):
        raise RuntimeError('Idle confirmation missing; inspect final_axes')
    print(json.dumps({'result': result, 'final_states': {n: a.get('state') for n, a in axes.items()}}))


if __name__ == '__main__': main()
