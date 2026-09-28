"""Exercise Jazzy diff_drive_controller against GenericSystem, never real CAN.

Start control_mock.launch.py in ROS_DOMAIN_ID=43 before running this script.
"""
import json
import math
import os
import time
import xml.etree.ElementTree as ET

if os.environ.get('ROS_DOMAIN_ID') != '43':
    raise SystemExit('Mock test requires ROS_DOMAIN_ID=43')

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('mrbobus_mock_check')
latest = {}
subs = [
    node.create_subscription(Odometry, '/diff_drive_controller/odom',
                             lambda msg: latest.update(odom=msg), 10),
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: latest.update(joints=msg), 10),
    node.create_subscription(String, '/robot_description',
                             lambda msg: latest.update(description=msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
]
pub = node.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)


def spin_until(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('Expected controller state not received within timeout')


def drive(linear, angular, duration):
    end = time.monotonic() + duration
    while time.monotonic() < end:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)


def velocities():
    msg = latest['joints']
    return dict(zip(msg.name, msg.velocity))


def yaw():
    q = latest['odom'].pose.pose.orientation
    return 2.0 * math.atan2(q.z, q.w)


try:
    spin_until(lambda: all(k in latest for k in ('description', 'odom', 'joints')) and pub.get_subscription_count() > 0)
    plugins = [e.text for e in ET.fromstring(latest['description']).findall('ros2_control/hardware/plugin')]
    assert plugins == ['mock_components/GenericSystem'], f'Refusing non-mock hardware: {plugins}'
    names = set(velocities())
    expected = {f'{side}_wheel_joint' for side in ('rear_left', 'rear_right', 'front_left', 'front_right')}
    assert names == expected, names
    initial_x = latest['odom'].pose.pose.position.x
    drive(0.05, 0.0, 2.0)
    spin_until(lambda: all(v > 0.1 for v in velocities().values()))
    forward = velocities()
    displacement = latest['odom'].pose.pose.position.x - initial_x
    assert displacement > 0.04, displacement
    # Stop publishing entirely: exercise the controller's command timeout.
    stopped_at = time.monotonic()
    spin_until(lambda: all(abs(v) < 1e-4 for v in velocities().values()), timeout=2.0)
    timeout_stop_s = time.monotonic() - stopped_at
    initial_yaw = yaw()
    drive(0.0, 0.3, 2.0)
    spin_until(lambda: velocities()['front_left_wheel_joint'] < -0.1 and velocities()['front_right_wheel_joint'] > 0.1)
    turning = velocities()
    turn = math.atan2(math.sin(yaw() - initial_yaw), math.cos(yaw() - initial_yaw))
    assert turn > 0.2, turn
    drive(0.0, 0.0, 0.8)
    spin_until(lambda: all(abs(v) < 1e-4 for v in velocities().values()))
    print(json.dumps({'forward_velocity_rad_s': forward, 'forward_displacement_m': displacement,
                      'timeout_stop_s': timeout_stop_s, 'turn_velocity_rad_s': turning,
                      'yaw_change_rad': turn, 'result': 'PASS'}, indent=2))
finally:
    node.destroy_node()
    rclpy.shutdown()
