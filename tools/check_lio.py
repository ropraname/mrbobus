#!/usr/bin/env python3
"""Passive L2/LIO timing and stationary drift check; never commands motion."""
import argparse
import json
import math
import statistics
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('check_lio_passive')
    counts, ages, poses, clouds, acc = {}, {}, [], [], []
    prev, regressions, gyro = {}, {}, []

    def callback(name, msg):
        counts[name] = counts.get(name, 0) + 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ages.setdefault(name, []).append(node.get_clock().now().nanoseconds * 1e-9 - stamp)
        if stamp <= prev.get(name, -math.inf):
            regressions[name] = regressions.get(name, 0) + 1
        prev[name] = stamp
        if name == 'lio':
            p = msg.pose.pose.position
            poses.append((p.x, p.y, p.z))
        elif name == 'imu':
            a = msg.linear_acceleration
            acc.append((a.x, a.y, a.z))
            g = msg.angular_velocity
            gyro.append((g.x, g.y, g.z))
        elif not clouds:
            fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
            result = {'frame': msg.header.frame_id, 'points': msg.width * msg.height,
                      'fields': fields}
            if 'time' in fields and fields['time'][1] == 7:
                times = [struct.unpack_from('>f' if msg.is_bigendian else '<f', msg.data,
                         i * msg.point_step + fields['time'][0])[0]
                         for i in range(msg.width * msg.height)]
                result['point_time_range_seconds'] = [min(times), max(times)]
            clouds.append(result)

    subscriptions = [node.create_subscription(typ, topic,
        lambda msg, n=name: callback(n, msg), qos_profile_sensor_data)
        for name, topic, typ in [('cloud', '/unilidar/cloud', PointCloud2),
                                ('imu', '/unilidar/imu', Imu),
                                ('lio', '/lio/odometry', Odometry)]]
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = {'seconds': time.monotonic() - start, 'counts': counts,
              'stamp_regressions': regressions, 'cloud': clouds,
              'age_seconds': {n: {'median': statistics.median(a), 'max': max(a)} for n, a in ages.items()}}
    if acc:
        result['mean_acceleration'] = [statistics.mean(a[i] for a in acc) for i in range(3)]
        result['mean_angular_velocity'] = [statistics.mean(g[i] for g in gyro) for i in range(3)]
    if poses:
        result['lio_first_xyz'] = poses[0]
        result['lio_last_xyz'] = poses[-1]
        result['lio_max_displacement_m'] = max(math.dist(poses[0], p) for p in poses)
        result['lio_all_finite'] = all(math.isfinite(v) for p in poses for v in p)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
