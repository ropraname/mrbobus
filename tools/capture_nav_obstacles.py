"""Read-only synchronized Nav2 evidence; never publishes commands or touches CAN."""
import argparse,json,time,math,urllib.request
from pathlib import Path
from datetime import datetime
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer,TransformListener

p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=180);p.add_argument('--url',default='http://192.168.67.149:8080');args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
rclpy.init();node=rclpy.create_node('nav_obstacle_evidence');buffer=Buffer();listener=TransformListener(buffer,node);latest={}
def receive(key,msg):latest[key]=(time.time(),msg)
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
subs=[]
for key,topic in [('local','/local_costmap/costmap'),('global','/global_costmap/costmap')]:
 subs.append(node.create_subscription(OccupancyGrid,topic,lambda m,k=key:receive(k,m),qos))
subs.append(node.create_subscription(LaserScan,'/navigation/scan',lambda m:receive('scan',m),rclpy.qos.qos_profile_sensor_data))
subs.append(node.create_subscription(RosPath,'/plan',lambda m:receive('plan',m),10))
def fetch(path):
 with urllib.request.urlopen(args.url+path,timeout=2) as r:return r.read()
def pose(p):return {'position':[p.position.x,p.position.y,p.position.z],'orientation':[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]}
end=time.monotonic()+args.seconds;nextshot=0
try:
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if time.monotonic()<nextshot:continue
  nextshot=time.monotonic()+1
  now=time.time();data={'time':datetime.now().astimezone().isoformat(),'received_at':now,'layers':{}}
  try:
   data['status']=json.loads(fetch('/api/status'));data['status'].pop('trace',None)
   jpeg=fetch('/camera.jpg');data['camera_received_at']=time.time()
  except Exception as e:print(str(e),flush=True);continue
  for key,(at,m) in latest.items():
   layer={'received_age':now-at,'frame':m.header.frame_id,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9}
   try:
    t=buffer.lookup_transform(m.header.frame_id,'base_link',rclpy.time.Time()).transform
    layer['robot']={'translation':[t.translation.x,t.translation.y,t.translation.z],'rotation':[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]}
   except Exception as e:layer['tf_error']=str(e)
   if key in ('local','global'):layer.update(width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin=pose(m.info.origin),data=list(m.data))
   elif key=='scan':layer.update(angle_min=m.angle_min,angle_increment=m.angle_increment,ranges=[r if math.isfinite(r) else None for r in m.ranges])
   else:layer['path']=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
   data['layers'][key]=layer
  name=datetime.now().strftime('%H%M%S-%f');(out/(name+'.jpg')).write_bytes(jpeg);(out/(name+'.json')).write_text(json.dumps(data,ensure_ascii=False))
  print(name,data['status'].get('navigation',{}).get('active'),data['status'].get('navigation',{}).get('message'),list(data['layers']),flush=True)
finally:node.destroy_node();rclpy.shutdown()
