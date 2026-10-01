"""Nav2 action adapter. Manual commands and STOP always cancel navigation."""
import math,time,threading,subprocess,copy
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped,TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from .radio import allow_source
from .ground import separate_ground,floor_clearing_ranges,traversable_lawn_mask

class Navigation:
    def __init__(self,node):
        self.floor_frames=[];self.result_status=None;self.ground_time=0.;self.ground_status={};self.node=node;self.goal=None;self.active=False;self.token=0;self.message='Цель не задана';self.path=[]
        self.state_client=node.create_client(GetState,'/bt_navigator/get_state')
        self.client=ActionClient(node,NavigateToPose,'/navigate_to_pose');self.tf=TransformBroadcaster(node)
        self.scan_pub=node.create_publisher(LaserScan,'/navigation/scan',qos_profile_sensor_data)
        self.floor_pub=node.create_publisher(LaserScan,'/navigation/floor_clear',qos_profile_sensor_data)
        self.sub=node.create_subscription(TwistStamped,'/navigation/cmd_vel',self.velocity,10)
        self.path_sub=node.create_subscription(Path,'/plan',self.plan,10)
        self.timer=node.create_timer(.05,self.broadcast)
    def plan(self,msg):self.path=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses]
    def cancel(self):
        self.result_status=None;self.token+=1;self.active=False;self.message='Остановлено'
        if self.goal:self.goal.cancel_goal_async();self.goal=None
    def broadcast(self):
        n=self.node
        with n.lock:
            if n.pose_source!='LIO' or time.monotonic()-n.pose_time>.3 or n.wheel_pose is None:return
            wheel=n.wheel_pose;age=n.get_clock().now().nanoseconds/1e9-wheel.header.stamp.sec-wheel.header.stamp.nanosec/1e9
            if not -.1<age<.3:return
            current=n.pose_matrix.copy()
        with n.field_map.lock:
            if n.field_map.alignment is None:return
            from .server import matrix
            m=n.field_map.alignment@current@np.linalg.inv(matrix(wheel.pose.pose.position,wheel.pose.pose.orientation))
        q=Rotation.from_matrix(m[:3,:3]).as_quat();t=TransformStamped();t.header.stamp=wheel.header.stamp;t.header.frame_id='map';t.child_frame_id='odom'
        t.transform.translation.x=float(m[0,3]);t.transform.translation.y=float(m[1,3]);t.transform.translation.z=float(m[2,3]);t.transform.rotation.x=float(q[0]);t.transform.rotation.y=float(q[1]);t.transform.rotation.z=float(q[2]);t.transform.rotation.w=float(q[3]);self.tf.sendTransform(t)
    def scan(self,p,stamp):
        # Obstacles and clearing remain current-scan only. Accumulate only the
        # floor fit, compensated by fresh LIO, to cover the L2 scan pattern.
        now=time.monotonic();floor_points=None;n=self.node
        with n.lock:
            current=n.pose_matrix.copy() if n.pose_source=='LIO' and now-n.pose_time<.2 and n.pose_matrix is not None else None
        if current is not None:
            samples=p[(p[:,2]>-.35)&(p[:,2]<.02)][::2]
            self.floor_frames=[(t,pts) for t,pts in self.floor_frames if now-t<.5]
            self.floor_frames.append((now,samples@current[:3,:3].T+current[:3,3]))
            floor_points=(np.concatenate([pts for _,pts in self.floor_frames])-current[:3,3])@current[:3,:3]
        else:self.floor_frames=[]
        try:
            q,self.ground_status=separate_ground(p,floor_points);self.ground_time=now
        except ValueError as e:
            self.ground_status={'error':str(e)};return
        extra_floor=None
        with n.field_map.lock:
            alignment=n.field_map.alignment.copy() if n.field_map.alignment is not None and n.field_map.quality and n.field_map.quality.get('verified') else None
        if current is not None and alignment is not None:
            tower=next(o for o in n.mission.config['objects'] if o['id']=='round_tower')
            lawn=next(a for a in tower['surroundings'] if a['id']=='round_tower_lawn')
            areas=[lawn]+[a for a in n.mission.config.get('special_areas',[]) if a['type']=='bridge']
            frame=alignment@current;plane=self.ground_status['plane']
            low=np.zeros(len(p),bool);removable=np.zeros(len(q),bool)
            for area in areas:
                xy=np.asarray(area['footprint']);bounds=(xy[:,0].min(),xy[:,0].max(),xy[:,1].min(),xy[:,1].max())
                low|=traversable_lawn_mask(p,plane,frame,bounds)
                removable|=traversable_lawn_mask(q,plane,frame,bounds)
            extra_floor=p[low]
            self.ground_status['traversable_surface_returns']=int(removable.sum())
            q=q[~removable]
        ranges=np.full(360,np.inf,dtype=np.float32)
        if len(q):
            distance=np.linalg.norm(q[:,:2],axis=1);idx=np.floor((np.arctan2(q[:,1],q[:,0])+math.pi)/(2*math.pi)*360).astype(int)%360;np.minimum.at(ranges,idx,distance)
        msg=LaserScan();msg.header.stamp=stamp;msg.header.frame_id='base_link';msg.angle_min=-math.pi;msg.angle_max=math.pi-2*math.pi/360;msg.angle_increment=2*math.pi/360;msg.range_min=.22;msg.range_max=8.;msg.scan_time=1/12;msg.ranges=ranges.tolist();self.scan_pub.publish(msg)
        clear=copy.deepcopy(msg);clear.ranges=floor_clearing_ranges(p,self.ground_status['plane'],q,extra_floor).tolist();self.floor_pub.publish(clear)
    def velocity(self,msg):
        n=self.node
        with n.gate.lock:
            if not self.active or not n.gate.active:return
            if not allow_source():self.cancel();n.zero();return
            now=time.monotonic();age=n.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
            if not -.1<=age<=.3 or now-n.lio_time>.3 or now-n.cloud_time>.3 or now-self.ground_time>.3:
                n.zero()
                # No stale command is sent. Brief gaps pause instead of aborting
                # the whole mission; sustained sensor loss still cancels it.
                if max(now-n.lio_time,now-n.cloud_time,now-self.ground_time)>.9:self.cancel()
                return
            with n.lock:
                axes_ok=len(n.axes)==4 and all(a['state']==8 and not a['error'] and now-a['at']<.5 for a in n.axes.values())
            if not axes_ok:self.cancel();n.zero();return
            v,w=msg.twist.linear.x,msg.twist.angular.z
            if not math.isfinite(v) or not math.isfinite(w):self.cancel();n.zero();return
            out=TwistStamped();out.header.stamp=n.get_clock().now().to_msg();out.twist.linear.x=max(-.30,min(.30,v));out.twist.angular.z=max(-.75,min(.75,w));n.pub.publish(out)
    def navigate(self,data):
        n=self.node
        with n.gate.lock:
            if not allow_source():raise ValueError('Для навигации нужен AUTO и снятый STOP')
            if not n.gate.active or n.gate.owner!=data.get('client'):raise ValueError('Включи привод в этом пульте')
            if time.monotonic()-n.lio_time>.3:raise ValueError('Нет свежей локализации')
            with n.field_map.lock:
                if not n.field_map.quality or not n.field_map.quality.get('verified'):raise ValueError('Сначала STOP, поставь робота на карте и уточни по лидару')
                goal=next((g for g in n.field_map.goals if g['id']==data.get('id')),None)
            if not goal:raise ValueError('Цель не найдена')
            self.cancel();token=self.token;n.zero()
        subprocess.run(['sudo','-n','systemctl','start','mrbobus-navigation'],check=True,timeout=5)
        if not self.client.wait_for_server(timeout_sec=10):raise ValueError('Nav2 ещё запускается или ждёт TF. Повтори через несколько секунд.')
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            if self.state_client.wait_for_service(timeout_sec=.2):
                f=self.state_client.call_async(GetState.Request());event=threading.Event();f.add_done_callback(lambda _:event.set())
                if event.wait(1) and f.result().current_state.id==3:break
            time.sleep(.1)
        else:raise ValueError('Nav2 не активирован: проверь позу и TF')
        req=NavigateToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=n.get_clock().now().to_msg();req.pose.pose.position.x=goal['x'];req.pose.pose.position.y=goal['y'];req.pose.pose.orientation.z=math.sin(goal['yaw']/2);req.pose.pose.orientation.w=math.cos(goal['yaw']/2)
        with n.gate.lock:
            if token!=self.token or not n.gate.active:raise ValueError('Цель отменена')
            future=self.client.send_goal_async(req)
        event=threading.Event();future.add_done_callback(lambda _:event.set())
        if not event.wait(5):
            self.cancel()
            future.add_done_callback(lambda f:f.result().cancel_goal_async() if f.result().accepted else None)
            raise ValueError('Nav2 не подтвердил цель')
        handle=future.result()
        with n.gate.lock:
            if token!=self.token or not n.gate.active:handle.cancel_goal_async();raise ValueError('Цель отменена')
            if not handle.accepted:raise ValueError('Nav2 отклонил цель')
            self.goal=handle;self.active=True;self.message='Движение к '+goal['label']
        def done(f):
            with n.gate.lock:
                if token!=self.token:return
                self.result_status=f.result().status;self.active=False;self.goal=None;self.message='Цель достигнута' if f.result().status==4 else 'Цель остановлена/недостижима';n.zero()
        handle.get_result_async().add_done_callback(done)
        return {'ok':True}
