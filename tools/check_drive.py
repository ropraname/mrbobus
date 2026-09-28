"""Gentle suspended-wheel check, explicitly opted into with --suspended-wheels.

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

parser = argparse.ArgumentParser()
parser.add_argument('--suspended-wheels', action='store_true', required=True)
parser.parse_args()
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
subs = [node.create_subscription(Odometry, '/odom', lambda m: latest.update(odom=m), 10),
        node.create_subscription(JointState, '/joint_states', lambda m: latest.update(joints=m), 10)]
can = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
can.bind(('can0',)); can.setblocking(False)
history = {n: deque() for n in range(4)}
average_peak = {n: 0. for n in range(4)}
axes = {}; peak = {n: 0. for n in range(4)}; samples = []

def poll():
    rclpy.spin_once(node, timeout_sec=.005)
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

def command(v):
    m = TwistStamped(); m.header.stamp = node.get_clock().now().to_msg()
    m.twist.linear.x = v; pub.publish(m)

def run(v, duration, monitor=True):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        command(v); poll()
        samples.append({str(n): dict(a) for n, a in axes.items()})
        if monitor:
            now = time.monotonic()
            for n in range(4):
                a = axes.get(n, {})
                if now-a.get('heartbeat', 0) > .5 or now-a.get('feedback', 0) > .5:
                    raise RuntimeError('Stale axis ' + str(n))
                if a.get('state') != 8 or a.get('error') != 0:
                    raise RuntimeError('Axis fault: ' + str(axes))
                if not math.isfinite(a['velocity']) or abs(a['velocity']) > 1. or a.get('average_rps', 0.) > .1:
                    raise RuntimeError('Wheel speed guard (60 rpm instantaneous / 6 rpm averaged): ' + str(axes))
        time.sleep(.035)

armed = False
try:
    controllers = service('list_controllers', ListControllers, ListControllers.Request())
    if not any(c.name == 'diff_drive_controller' and c.state == 'inactive' for c in controllers.controller):
        raise RuntimeError('Drive must start inactive')
    end = time.monotonic() + .5
    while time.monotonic() < end: poll()
    if len(axes) != 4 or any(a.get('state') != 1 for a in axes.values()):
        raise RuntimeError('All four axes must start Idle: ' + str(axes))
    request = SwitchController.Request(); request.activate_controllers = ['diff_drive_controller']; request.strictness = 2
    armed = True
    if not service('switch_controller', SwitchController, request).ok: raise RuntimeError('Activation failed')
    run(0., 1., monitor=False)
    run(.007, 5.)  # ~1.5 rpm; controller acceleration limit gives 1 s ramp.
    run(0., 2.)
    result = {'peak_average_rpm': {n: v*60 for n, v in average_peak.items()}, 'peak_measured_rpm': {n: v*60 for n, v in peak.items()}, 'last_axes': axes,
              'odom_x': latest['odom'].pose.pose.position.x if 'odom' in latest else None,
              'sample_count': len(samples)}
    with open('/tmp/mrbobus-drive-samples.json', 'w') as f: json.dump(samples, f)
    print(json.dumps(result, indent=2))
finally:
    with open('/tmp/mrbobus-drive-samples.json', 'w') as f: json.dump(samples, f)
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
