"""Nav2 smoke test in isolated ROS domain45; no CAN or physical drive publisher."""
import os,time,math,subprocess,signal,threading
import numpy as np
if os.environ.get('ROS_DOMAIN_ID')!='45':raise SystemExit('Requires isolated domain45')
import rclpy
from geometry_msgs.msg import TwistStamped,TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
rclpy.init();n=rclpy.create_node('navigation_simulated_base');a=np.load('/home/taras/robot/records/maps/field-20260928/field.npz');path=a['trajectory'];start=path[0];goal=next(p for p in path if np.linalg.norm(p[:2]-start[:2])>.45);yaw=math.atan2(goal[1]-start[1],goal[0]-start[0]);pose=[float(start[0]),float(start[1]),yaw];velocity=[0.,0.];commands=[]
pub=n.create_publisher(Odometry,'/odom',10);scan=n.create_publisher(LaserScan,'/navigation/scan',10);tf=TransformBroadcaster(n);static=StaticTransformBroadcaster(n)
t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;static.sendTransform(t)
def cmd(m):velocity[:]=[m.twist.linear.x,m.twist.angular.z];commands.append(velocity[:])
sub=n.create_subscription(TwistStamped,'/navigation/cmd_vel',cmd,10)
def tick():
 pose[0]+=.05*velocity[0]*math.cos(pose[2]);pose[1]+=.05*velocity[0]*math.sin(pose[2]);pose[2]+=.05*velocity[1]
 stamp=n.get_clock().now().to_msg();m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.position.x=pose[0];m.pose.pose.position.y=pose[1];m.pose.pose.orientation.z=math.sin(pose[2]/2);m.pose.pose.orientation.w=math.cos(pose[2]/2);m.twist.twist.linear.x=velocity[0];m.twist.twist.angular.z=velocity[1];pub.publish(m)
 t=TransformStamped();t.header=m.header;t.child_frame_id='base_link';t.transform.translation.x=pose[0];t.transform.translation.y=pose[1];t.transform.rotation=m.pose.pose.orientation;tf.sendTransform(t)
 s=LaserScan();s.header.stamp=stamp;s.header.frame_id='base_link';s.angle_min=-math.pi;s.angle_max=math.pi;s.angle_increment=math.pi/180;s.range_min=.3;s.range_max=8.;s.ranges=[float('inf')]*361;scan.publish(s)
timer=n.create_timer(.05,tick);thread=threading.Thread(target=rclpy.spin,args=(n,),daemon=True);thread.start();log=open('/tmp/mrbobus-nav2-smoke.log','w');proc=subprocess.Popen(['ros2','launch','/home/taras/robot/mrbobus_ws/deploy/navigation.launch.py'],stdout=log,stderr=log,start_new_session=True)
def wait(f,seconds):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:time.sleep(.1)
 if not f.done():raise RuntimeError('Timeout')
 return f.result()
try:
 client=ActionClient(n,NavigateToPose,'/navigate_to_pose');assert client.wait_for_server(timeout_sec=45),'Nav2 server did not activate'
 state_client=n.create_client(GetState,'/bt_navigator/get_state');assert state_client.wait_for_service(timeout_sec=10)
 end=time.monotonic()+30
 while wait(state_client.call_async(GetState.Request()),5).current_state.id!=3:
  assert time.monotonic()<end,'Navigator lifecycle timeout'
  time.sleep(.2)
 req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=n.get_clock().now().to_msg();req.pose.pose.position.x=float(goal[0]);req.pose.pose.position.y=float(goal[1]);req.pose.pose.orientation.z=math.sin(yaw/2);req.pose.pose.orientation.w=math.cos(yaw/2)
 h=wait(client.send_goal_async(req),5);assert h.accepted;result=wait(h.get_result_async(),40);print('RESULT',result.status,'COMMANDS',len(commands),'MAX_V',max((abs(x[0]) for x in commands),default=0),'END',pose,flush=True);assert result.status==4
finally:
 os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 rclpy.shutdown();thread.join(timeout=2)
