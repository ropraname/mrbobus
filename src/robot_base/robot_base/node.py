import fcntl
import json
import math
import os
import queue
import secrets
import socket
import struct
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from .core import Base, SIGNS, NAMES, frame
from .web import Panel

class RobotBase(Node):
    def __init__(self):
        super().__init__('robot_base')
        def param(name, default):
            return self.declare_parameter(name, default).value
        interface = param('can_interface', 'can0')
        motion = param('motion_enabled', False)
        limit = param('session_limit_s', 0.)
        self.ros_control = param('allow_ros_control', False)
        # Refuse a second cooperating driver process on the same interface.
        lock_dir = Path(os.environ.get('ROBOT_RUNTIME', '/tmp'))
        self.lock = open(lock_dir/f'robot-base-{interface}.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))
        self.sock.setblocking(False)
        self.base = Base(self.send, time.monotonic, motion, limit)
        self.events = queue.Queue(maxsize=32)
        self.emergency = threading.Event()
        self.emergency_reason = 'Panel emergency STOP'
        self.snapshot_data = self.base.status(time.monotonic())
        self.ros_stamp = -1
        self.ros_seq = 0
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joints_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.tf = TransformBroadcaster(self)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TwistStamped, '/cmd_vel', self.velocity, qos)
        self.create_service(Trigger, '/robot/arm', self.arm_ros)
        self.create_service(Trigger, '/robot/stop', self.stop_ros)
        self.create_service(Trigger, '/robot/reset_fault', self.reset_ros)
        self.panel = None
        if param('web_enabled', True):
            share = Path(get_package_share_directory('robot_base'))
            self.panel = Panel(param('web_address','192.168.67.149'), param('web_port',8443),
                param('password_file','/home/taras/robot/secrets/panel-password'),
                param('cert_file','/home/taras/robot/secrets/panel.crt'),
                param('key_file','/home/taras/robot/secrets/panel.key'),
                share/'web', self.dispatch, lambda: self.snapshot_data,
                require_password=param('web_require_password', True))
        self.create_timer(.05, self.step)
        self.create_timer(.5, self.diagnostics)
        self.get_logger().info(f'CAN {interface}; motion_enabled={motion}; starts DISARMED')

    def send(self, raw):
        self.sock.send(raw)
        time.sleep(.0015)  # bounded pacing for MCP2515 TX queue

    def dispatch(self, op, owner, data):
        if op == 'stop' and data.get('emergency'):
            self.emergency_reason = 'Panel emergency STOP'
            self.emergency.set()
            return {'ok':True, 'message':'Emergency stop requested'}
        event = {'op':op, 'owner':owner, 'data':data, 'time':time.monotonic(),
                 'done':threading.Event(), 'response':None}
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.emergency_reason = 'Panel command queue full'
            self.emergency.set()
            return {'ok':False,'error':'Command queue full; stopping'}
        if not event['done'].wait(.4):
            self.emergency_reason = 'Panel request processing timeout'
            self.emergency.set()
            return {'ok':False,'error':'Control loop timeout; stopping'}
        return event['response']

    def arm_ros(self, request, response):
        if not self.ros_control:
            response.success, response.message = False, 'ROS control disabled; use authenticated panel'
        else:
            response.success, response.message = self.base.arm('ros', time.monotonic())
            self.ros_stamp = -1
        return response

    def stop_ros(self, request, response):
        self.base.fault('ROS emergency stop', time.monotonic())
        response.success, response.message = True, 'Stop latched'
        return response

    def reset_ros(self, request, response):
        if not self.ros_control:
            response.success, response.message = False, 'Use authenticated panel'
        else:
            response.success, response.message = self.base.reset(time.monotonic())
        return response

    def velocity(self, msg):
        if not self.ros_control or self.base.owner != 'ros':
            return
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= self.ros_stamp or not -.05 <= age <= .25 or msg.header.frame_id not in ('','base_link'):
            self.base.stop('Invalid/stale ROS command timestamp', time.monotonic())
            return
        if any(v != 0 for v in (msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y)):
            self.base.stop('Unsupported velocity components', time.monotonic())
            return
        self.ros_stamp, self.ros_seq = stamp, self.ros_seq+1
        self.base.command('ros',self.ros_seq,msg.twist.linear.x,msg.twist.angular.z,time.monotonic())

    def step(self):
        now = time.monotonic()
        try:
            for _ in range(128):
                try:
                    raw = self.sock.recv(16)
                except BlockingIOError:
                    break
                self.base.receive(raw, time.monotonic())
            if self.emergency.is_set():
                self.emergency.clear()
                self.base.fault(self.emergency_reason, now)
            for _ in range(32):
                try:
                    e = self.events.get_nowait()
                except queue.Empty:
                    break
                ok, message, lease = False, 'Expired request', None
                if now-e['time'] < .25:
                    op, owner, data = e['op'], e['owner'], e['data']
                    if op == 'arm':
                        lease = secrets.token_urlsafe(18)
                        ok, message = self.base.arm(owner+':'+lease, now)
                    elif op == 'command':
                        lease_value = data.get('lease','')
                        ok = isinstance(lease_value,str) and self.base.command(owner+':'+lease_value,data.get('seq'),data.get('linear'),data.get('angular'),now)
                        message = 'Accepted' if ok else 'Not owner / not armed / stale sequence'
                    elif op == 'stop':
                        # A background tab must not release another operator's hold.
                        # Explicit emergency STOP above remains available to everyone.
                        if self.base.owner == owner+':'+data.get('lease',''):
                            self.base.stop('Deadman released', now)
                        ok, message = True, 'Stopping'
                    elif op == 'reset':
                        ok, message = self.base.reset(now)
                e['response'] = {'ok':ok,'message':message}
                if ok and lease is not None:
                    e['response']['lease'] = lease
                e['done'].set()
            self.base.tick(time.monotonic())
            self.snapshot_data = self.base.status(time.monotonic())
            self.publish_motion()
        except Exception as exc:
            self.base.fault(f'CAN/control exception: {type(exc).__name__}: {exc}',time.monotonic())
            self.snapshot_data = self.base.status(time.monotonic())
            self.get_logger().error(self.base.reason)
            # Do not keep serving requests after transport failure; watchdog is fallback.
            self.shutdown_can()
            raise

    def publish_motion(self):
        stamp = self.get_clock().now().to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [NAMES[n]+'_joint' for n in SIGNS]
        joints.position = [self.base.axes[n].turns*2*math.pi for n in SIGNS]
        self.joints_pub.publish(joints)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.base.x, self.base.y
        odom.pose.pose.orientation.z = math.sin(self.base.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.base.yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.base.v, self.base.w
        # Conservative provisional uncertainty, NOT calibrated covariance.
        for index,value in zip((0,7,14,21,28,35),(.05,.05,1e6,1e6,1e6,.25)):
            odom.pose.covariance[index] = value if self.base.odom_valid else 1e6
            odom.twist.covariance[index] = value if self.base.odom_valid else 1e6
        self.odom_pub.publish(odom)
        if self.base.odom_valid:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = odom.header, 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.base.x,self.base.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)

    def diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = self.base.status(time.monotonic())
        d = DiagnosticStatus()
        d.name, d.hardware_id, d.message = 'robot_base','odrive-can-0-3',self.base.reason
        d.level = DiagnosticStatus.ERROR if self.base.state == 'FAULT' else (DiagnosticStatus.OK if self.base.fresh(time.monotonic()) else DiagnosticStatus.WARN)
        d.values = [KeyValue(key=str(k),value=json.dumps(v)) for k,v in status.items()]
        msg.status = [d]
        self.diag_pub.publish(msg)

    def shutdown_can(self):
        for n in SIGNS:
            try:
                self.send(frame(n,0x0D,struct.pack('<ff',0,0)))
                self.send(frame(n,7,struct.pack('<I',1)))
            except OSError:
                pass

def main():
    rclpy.init()
    node = None
    try:
        node = RobotBase()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.shutdown_can()
            if node.panel:
                node.panel.close()
            node.sock.close()
            node.lock.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
