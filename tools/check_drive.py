"""Gentle bounded drive check, explicitly selected as suspended or floor.

Requires the candidate control.launch.py already running on domain42/can0,
with diff_drive_controller inactive. Leaves the controller inactive.
"""
import argparse
from collections import deque
import json
import math
import os
import socket
import struct
import time
import urllib.request
from pathlib import Path

parser = argparse.ArgumentParser()
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument('--suspended-wheels', action='store_true')
mode.add_argument('--floor', action='store_true')
mode.add_argument('--stop-check', action='store_true')
parser.add_argument('--output', default='/tmp/mrbobus-drive-samples.json')
parser.add_argument('--reverse', action='store_true')
parser.add_argument('--require-lio', action='store_true')
parser.add_argument('--arc-pair', action='store_true', help='20 deg arc left and return heading')
parser.add_argument('--turn-pair', action='store_true', help='30 deg left, then return heading; no linear command')
parser.add_argument('--speed', type=float, default=.03)
parser.add_argument('--distance', type=float, default=.5)
args = parser.parse_args()
Path(args.output).parent.mkdir(parents=True, exist_ok=True)
if not math.isfinite(args.speed) or not 0 < args.speed <= .06: raise SystemExit('Speed must be in (0, 0.06] m/s')
if not math.isfinite(args.distance) or not 0 < args.distance <= 2.4: raise SystemExit('Distance must be in (0, 2.4] m')
if os.environ.get('ROS_DOMAIN_ID') != '42':
    raise SystemExit('Requires robot ROS_DOMAIN_ID=42')
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('gentle_drive_check')
pub = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
latest = {}
subs = [node.create_subscription(Odometry, '/odom', lambda m: latest.update(odom=m), 1),
        node.create_subscription(JointState, '/joint_states', lambda m: latest.update(joints=m), 1),
        node.create_subscription(Odometry, '/lio/odometry', lambda m: latest.update(lio=m), 1)]
can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
can.bind(('can0',)); can.setblocking(False)
history = {n: deque() for n in range(4)}
average_peak = {n: 0. for n in range(4)}
axes = {}; peak = {n: 0. for n in range(4)}; samples = []

def poll():
    rclpy.spin_once(node, timeout_sec=.005)
    for _ in range(4): rclpy.spin_once(node, timeout_sec=0.)
    while True:
        try: raw = can.recv(16)
        except BlockingIOError: break
        ident, length, data = struct.unpack('=IB3x8s', raw)
        if ident & 0xe0000000 or length != 8: continue
        n, cmd = ident >> 5, ident & 31
        if n not in range(4): continue
        a = axes.setdefault(n, {})
        if cmd == 1:
            a.update(error=struct.unpack_from('<I', data)[0], state=data[4], heartbeat=time.monotonic())
        if cmd == 20:
            target, measured=struct.unpack('<ff',data)
            a.update(iq_target=target,iq_measured=measured,iq_time=time.monotonic())
        if cmd == 9:
            p, v = struct.unpack('<ff', data)
            a.update(position=p, velocity=v, feedback=time.monotonic())
            peak[n] = max(peak[n], abs(v))
            if a.get('state') == 8:
                h = history[n]; h.append((time.monotonic(), p))
                while len(h) > 1 and h[-1][0]-h[1][0] >= .5: h.popleft()
                if h[-1][0]-h[0][0] >= .5:
                    average = abs((h[-1][1]-h[0][1])/(h[-1][0]-h[0][0]))
                    a['average_rps'] = average
                    average_peak[n] = max(average_peak[n], average)
            else: history[n].clear()

def service(name, kind, request):
    client = node.create_client(kind, '/controller_manager/' + name)
    try:
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError(name + ' unavailable')
        future = client.call_async(request); deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline: poll()
        if not future.done(): raise RuntimeError(name + ' timeout')
        return future.result()
    finally: node.destroy_client(client)

def command(v, w=0.):
    m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg()
    m.twist.linear.x = v; m.twist.angular.z = w; pub.publish(m)

def run(v, duration, monitor=True, w=0.):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        command(v,w); poll()
        sample={str(n): dict(a) for n, a in axes.items()}
        for topic in ('odom','lio'):
            if topic in latest:
                msg=latest[topic];p=msg.pose.pose.position;q=msg.pose.pose.orientation
                sample[topic]={'t':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'x':p.x,'y':p.y,'z':p.z,
                              'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
        samples.append(sample)
        if monitor:
            for topic in (('odom','lio') if args.require_lio else ('odom',)):
                if topic not in sample or not all(math.isfinite(sample[topic][k]) for k in ('t','x','y','z','yaw')):
                    raise RuntimeError('Missing/nonfinite '+topic)
                age=node.get_clock().now().nanoseconds*1e-9-sample[topic]['t']
                if not -.1 <= age <= .5: raise RuntimeError('Stale '+topic)
            if args.floor and 'odom' in latest:
                p=latest['odom'].pose.pose.position
                if math.hypot(p.x, p.y) > (.60 if args.arc_pair else .10 if args.turn_pair else args.distance+.10) or abs(p.y)>(.30 if args.arc_pair else .10):
                    raise RuntimeError('Floor test odometry displacement/lateral limit')
                if abs(sample['odom']['yaw'])>(math.radians(40) if (args.turn_pair or args.arc_pair) else .25): raise RuntimeError('Unexpected turning > 14 degrees')
            now = time.monotonic()
            for n in range(4):
                a = axes.get(n, {})
                if now-a.get('heartbeat', 0) > .5 or now-a.get('feedback', 0) > .5:
                    raise RuntimeError('Stale axis ' + str(n))
                if a.get('state') != 8 or a.get('error') != 0:
                    raise RuntimeError('Axis fault: ' + str(axes))
                average_limit = 20./60. if args.floor else .1
                if not math.isfinite(a['velocity']) or abs(a['velocity']) > 1. or a.get('average_rps', 0.) > average_limit:
                    raise RuntimeError(f'Wheel speed guard (60 rpm instantaneous / {average_limit*60:g} rpm averaged): ' + str(axes))
        time.sleep(.035)

armed = False
try:
    deadline = time.monotonic()+15.
    while True:
        controllers = service('list_controllers', ListControllers, ListControllers.Request())
        drive = next((c for c in controllers.controller if c.name == 'diff_drive_controller'), None)
        if drive and drive.state == 'active': raise RuntimeError('Drive must start inactive')
        if drive and drive.state == 'inactive': break
        if time.monotonic()>deadline: raise RuntimeError('Drive did not become ready')
        poll(); time.sleep(.1)
    end = time.monotonic() + .5
    while time.monotonic() < end: poll()
    if len(axes) != 4 or any(a.get('state') != 1 for a in axes.values()):
        raise RuntimeError('All four axes must start Idle: ' + str(axes))
    if args.require_lio:
        if 'lio' not in latest: raise RuntimeError('LIO required before arm')
        stamp=latest['lio'].header.stamp
        if not -.1 <= node.get_clock().now().nanoseconds*1e-9-stamp.sec-stamp.nanosec*1e-9 <= .5:
            raise RuntimeError('LIO stale before arm')
    request = SwitchController.Request(); request.activate_controllers = ['diff_drive_controller']; request.strictness = 2
    armed = True
    if not service('switch_controller', SwitchController, request).ok: raise RuntimeError('Activation failed')
    run(0., 1., monitor=False)
    if args.stop_check:
        start=time.monotonic()
        request=urllib.request.Request('http://192.168.67.149:8081/stop', data=b'{}',
                    headers={'Origin':'http://192.168.67.149:8081','Content-Type':'application/json'})
        with urllib.request.urlopen(request, timeout=2) as response: response.read()
        while time.monotonic()-start < 1.:
            poll()
            if all(a.get('state')==1 for a in axes.values()): break
        if not all(a.get('state')==1 for a in axes.values()):
            raise RuntimeError('STOP did not report all Idle within 1s')
        print(json.dumps({'stop_to_all_idle_s':time.monotonic()-start}))
        raise SystemExit(0)
    if args.turn_pair or args.arc_pair:
        for target in (math.radians(20 if args.arc_pair else 30), 0.):
            deadline=time.monotonic()+15.
            while time.monotonic()<deadline:
                if 'odom' not in latest: raise RuntimeError('No odometry')
                q=latest['odom'].pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                error=target-yaw
                if abs(error)<.02: break
                w=math.copysign(min(.1 if args.arc_pair else .3,math.sqrt(2*.12*max(0.,abs(error)-.02))),error)
                run(.05 if args.arc_pair else 0.,.04,w=w)
            else: raise RuntimeError('Turn test 15 s timeout')
            run(0.,1.)
    elif args.floor:
        deadline=time.monotonic()+max(30.,args.distance/args.speed+15.)
        while time.monotonic()<deadline:
            if 'odom' not in latest: raise RuntimeError('No odometry')
            direction=-1. if args.reverse else 1.
            remaining=args.distance-direction*latest['odom'].pose.pose.position.x
            if remaining <= .005: break
            run(direction*min(args.speed, math.sqrt(2*.025*max(0.,remaining-.005))), .04)
        else: raise RuntimeError('Floor test time limit')
    else: run(.007, 5.)
    run(0., 2.)
    result = {'peak_average_rpm': {n: v*60 for n, v in average_peak.items()}, 'peak_measured_rpm': {n: v*60 for n, v in peak.items()}, 'last_axes': axes,
              'odom_x': latest['odom'].pose.pose.position.x if 'odom' in latest else None,
              'sample_count': len(samples), 'max_abs_yaw_deg': max((abs(x['odom']['yaw'])*180/math.pi for x in samples if 'odom' in x),default=0.)}
    Path(args.output).with_suffix('.summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
finally:
    with open(args.output, 'w') as f: json.dump(samples, f)
    if armed:
        command(0.)
        request = SwitchController.Request(); request.deactivate_controllers = ['diff_drive_controller']; request.strictness = 2
        try:
            response = service('switch_controller', SwitchController, request)
            print('Controller inactive:', response.ok)
        except Exception as error: print('Deactivate failed:', error)
        end = time.monotonic() + .4
        while time.monotonic() < end: poll()
        print('Final axis states:', {n: a.get('state') for n, a in axes.items()})
    can.close(); node.destroy_node(); rclpy.shutdown()
