"""Passive ROS topic/TF check; safe on either host. Never publishes commands."""
import collections
import json
import math
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu,PointCloud2,JointState
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage

rclpy.init();n=rclpy.create_node('robot_passive_check')
counts=collections.Counter();latest={};old={};bad=[];transforms={};acc=[]
def callback(name,msg):
    counts[name]+=1
    if hasattr(msg,'header'):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if stamp<=0 or stamp<old.get(name,0):bad.append(name+': timestamp')
        old[name]=stamp
        latest[name]=n.get_clock().now().nanoseconds/1e9-stamp
    if name=='imu':acc.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
    if name in ('tf','tf_static'):
        for tf in msg.transforms:transforms[tf.child_frame_id]=tf.header.frame_id

subs=[]
for topic,name,kind in [('/unilidar/cloud','cloud',PointCloud2),('/unilidar/imu','imu',Imu),
                       ('/joint_states','joints',JointState),('/odom','odom',Odometry),
                       ('/diagnostics','diagnostics',DiagnosticArray),('/tf','tf',TFMessage)]:
    subs.append(n.create_subscription(kind,topic,lambda m,k=name:callback(k,m),qos_profile_sensor_data))
subs.append(n.create_subscription(TFMessage,'/tf_static',lambda m:callback('tf_static',m),
    QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
end=time.monotonic()+10
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
mean=[sum(p[i] for p in acc)/len(acc) for i in range(3)] if acc else []
print(json.dumps({'counts':counts,'last_age_s':latest,'tf':transforms,'imu_mean':mean,'errors':bad},indent=2))
ok=all(counts[k]>5 for k in ('cloud','imu','joints','odom','diagnostics')) and not bad
ok &= all(k in transforms for k in ('base_link','front_left_wheel','unilidar_lidar','unilidar_imu'))
ok &= all(-.1<v<.5 for v in latest.values())
n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if ok else 1)
