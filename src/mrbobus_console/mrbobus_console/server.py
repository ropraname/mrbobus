"""Bounded LAN console: ROS commands only; CAN listener is strictly passive."""
import argparse,json,math,os,signal,socket,struct,subprocess,threading,time,uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.request import Request,urlopen
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,CompressedImage
from controller_manager_msgs.srv import ListControllers,SwitchController
from tf2_ros import Buffer,TransformListener
from .control import MotionGate
from .field_map import FieldMap
from .navigation import Navigation

ROOT=Path('/home/taras/robot/mrbobus_ws')
STOP_URL='http://192.168.67.149:8081'
def stop_request(path):
    req=Request(STOP_URL+path,data=b'{}',headers={'Origin':STOP_URL,'Content-Type':'application/json'})
    with urlopen(req,timeout=2) as response:return json.load(response)
def matrix(p,q):
    x,y,z,w=q.x,q.y,q.z,q.w
    m=np.eye(4);m[:3,:3]=[[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],[2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],[2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]]
    m[:3,3]=[p.x,p.y,p.z];return m

def transform_matrix(t):return matrix(t.transform.translation,t.transform.rotation)

class Console(Node):
    def __init__(self,args):
        super().__init__('mrbobus_console');self.args=args;self.gate=MotionGate();self.lock=threading.RLock();self.operation=threading.Lock();self.running=True
        self.field_map=FieldMap('/home/taras/robot/records/maps/field-20260928');self.iq={}
        self.pose=None;self.pose_source=None;self.pose_time=0.;self.pose_matrix=None;self.lio_time=0.;self.wheel_pose=None
        self.cloud_frames=[];self.cloud_stats={};self.points=[];self.cloud_time=0.;self.cloud_processed=0.;self.cloud_hz=0.;self.previous_cloud=None;self.trace=[];self.axes={};self.voltage={}
        self.jpeg=None;self.camera_time=0.;self.camera_error='Подключение камеры';self.camera_process=None;self.lio_process=None;self.recorder=None
        self.session=datetime.now().strftime('field-%Y%m%d-%H%M%S');self.folder=Path(args.records)/self.session;self.marks=[];self.segment=0;self.record_error=None
        self.pub=self.create_publisher(TwistStamped,'/diff_drive_controller/cmd_vel',10)
        self.navigation=Navigation(self)
        self.camera_pub=self.create_publisher(CompressedImage,'/camera/image/compressed',qos_profile_sensor_data)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.subs=[self.create_subscription(PointCloud2,'/unilidar/cloud',self.cloud,qos_profile_sensor_data),self.create_subscription(Odometry,'/lio/odometry',self.lio,qos_profile_sensor_data),self.create_subscription(Odometry,'/odom',self.wheel,qos_profile_sensor_data)]
        self.list_client=self.create_client(ListControllers,'/controller_manager/list_controllers');self.switch_client=self.create_client(SwitchController,'/controller_manager/switch_controller')
        threading.Thread(target=self.camera,daemon=True).start();threading.Thread(target=self.can_listener,daemon=True).start()
    def pose_set(self,m,source):
        with self.lock:
            if self.pose_source!=source:self.trace=[]
            self.pose_matrix=m;self.pose_source=source;self.pose_time=time.monotonic()
            self.pose={'x':float(m[0,3]),'y':float(m[1,3]),'z':float(m[2,3]),'yaw':math.atan2(m[1,0],m[0,0]),'frame':'lio_odom' if source=='LIO' else 'odom'}
            p=[round(self.pose['x'],3),round(self.pose['y'],3),round(self.pose['z'],3)]
            if not self.trace or math.dist(p,self.trace[-1])>.015:self.trace.append(p);self.trace=self.trace[-2500:]
    def lio(self,msg):
        if not all(math.isfinite(v) for v in (msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)):return
        try:
            # T_world_base = T_world_imu * T_imu_base; never publishes a second TF owner.
            t=self.tf.lookup_transform('unilidar_imu','base_link',Time())
            m=matrix(msg.pose.pose.position,msg.pose.pose.orientation)@transform_matrix(t)
            age=self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
            if not -.1<=age<=.5:return
            self.lio_time=time.monotonic();self.pose_set(m,'LIO')
        except Exception:pass
    def wheel(self,msg):
        self.wheel_pose=msg
        if time.monotonic()-self.lio_time>.7:self.pose_set(matrix(msg.pose.pose.position,msg.pose.pose.orientation),'колёсная')
    def cloud(self,msg):
        now=time.monotonic()
        if self.previous_cloud:
            hz=1/max(.001,now-self.previous_cloud);self.cloud_hz=.9*self.cloud_hz+.1*hz
        self.previous_cloud=now;self.cloud_time=now
        try:
            fs={f.name:f.offset for f in msg.fields};endian='>' if msg.is_bigendian else '<'
            dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fs[k] for k in ('x','y','z')],'itemsize':msg.point_step})
            a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
            p=np.stack([a[k] for k in ('x','y','z')],axis=1)
            raw=len(p);p=p[np.isfinite(p).all(axis=1)]
            radius=np.linalg.norm(p,axis=1);p=p[(radius>.3)&(radius<8)]
            t=transform_matrix(self.tf.lookup_transform('base_link',msg.header.frame_id,Time()));p=p@t[:3,:3].T+t[:3,3]
            self.navigation.scan(p,msg.header.stamp)
            # Show below the chassis too: the robot may stand on a table or ramp.
            p=p[(p[:,2]>-2.5)&(p[:,2]<3)]
            with self.lock:
                pose=self.pose_matrix.copy() if self.pose_matrix is not None else None
                stable=self.pose_source=='LIO' and now-self.pose_time<.2
            if stable:
                world=p@pose[:3,:3].T+pose[:3,3]
                self.cloud_frames=[(stamp,cloud) for stamp,cloud in self.cloud_frames if now-stamp<1.5]
                self.cloud_frames.append((now,world))
            else:self.cloud_frames=[]
            if now-self.cloud_processed<.25:return
            self.cloud_processed=now
            if self.cloud_frames:
                p=np.concatenate([cloud for _,cloud in self.cloud_frames])
                # Spatial sampling retains surfaces instead of dropping whole scan phases.
                _,indices=np.unique(np.floor(p/.04).astype(np.int32),axis=0,return_index=True)
                p=p[indices];p=(p-pose[:3,3])@pose[:3,:3]
            if len(p)>16000:p=p[np.linspace(0,len(p)-1,16000,dtype=int)]
            with self.lock:
                self.points=np.round(p,3).tolist()
                self.cloud_stats={'raw':raw,'shown':len(p),'window':1.5 if stable else 0,'voxel':.04}
        except Exception as e:
            self.get_logger().warning('Cloud display: '+str(e),throttle_duration_sec=10)
    def call(self,client,request,timeout=6):
        if not client.wait_for_service(timeout_sec=timeout):raise ValueError('Контроллер не отвечает')
        future=client.call_async(request);event=threading.Event();future.add_done_callback(lambda _:event.set())
        if not event.wait(timeout):raise ValueError('Таймаут контроллера')
        return future.result()
    def zero(self):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();self.pub.publish(msg)
    def stop(self):
        self.navigation.cancel();self.gate.stop();self.zero();stop_request('/stop')
        return {'ok':True}
    def arm(self,owner):
        with self.operation:
            epoch=self.gate.begin_arm(owner)
            try:
                # Diagnostics occur once per explicit arm, while the CAN owner is stopped.
                subprocess.run(['sudo','-n','systemctl','stop','mrbobus-control','robot-base'],check=True,timeout=6)
                subprocess.run(['sudo','-n','install','-d','-o','taras','-g','taras','/run/robot-base'],check=True,timeout=3)
                result=subprocess.run(['python3',str(ROOT/'tools/read_bus.py')],capture_output=True,text=True,timeout=3)
                if result.returncode:raise ValueError('Проверка осей/питания не пройдена: '+result.stdout[-160:]+result.stderr[-160:])
                self.gate.check_epoch(epoch)
                Path('/run/mrbobus-stop/control.env').write_text('DRIVE_GAIN_PROFILE=baseline\nDRIVE_CURRENT_LIMIT=20\n')
                stop_request('/ready');self.gate.check_epoch(epoch)
                subprocess.run(['sudo','-n','systemctl','start','mrbobus-control'],check=True,timeout=6)
                deadline=time.monotonic()+12
                while time.monotonic()<deadline:
                    self.gate.check_epoch(epoch)
                    result=self.call(self.list_client,ListControllers.Request(),max(.1,deadline-time.monotonic()))
                    state=next((x.state for x in result.controller if x.name=='diff_drive_controller'),None)
                    if state=='inactive':break
                    time.sleep(.1)
                else:raise ValueError('Контроллер не готов')
                self.zero();self.gate.check_epoch(epoch)
                req=SwitchController.Request();req.activate_controllers=['diff_drive_controller'];req.strictness=2
                if not self.call(self.switch_client,req).ok:raise ValueError('Не удалось включить привод')
                self.gate.finish_arm(epoch);return {'ok':True}
            except Exception:
                self.stop();raise
    def command(self,data):
        with self.gate.lock:
            self.navigation.cancel()
            v,w=self.gate.command(data.get('client'),data.get('v',0),data.get('w',0),data.get('seq'))
            if Path('/run/mrbobus-stop/enabled').read_text().strip()!='1':raise ValueError('STOP зафиксирован')
            with self.lock:
                if len(self.axes)!=4 or any(time.monotonic()-x['at']>.5 or x['state']!=8 or x['error'] for x in self.axes.values()):raise ValueError('Оси не готовы')
            msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.twist.linear.x=v;msg.twist.angular.z=w;self.pub.publish(msg)
        return {'ok':True}
    def start_lio(self):
        if time.monotonic()-self.lio_time<1:return
        if self.lio_process and self.lio_process.poll() is None:return
        existing=subprocess.run(['pgrep','-f','^/home/taras/robot/lio_ws/install/point_lio/lib/point_lio/pointlio_mapping'],stdout=subprocess.DEVNULL)
        if existing.returncode==0:return
        if self.gate.active:raise ValueError('Для инициализации LIO сначала останови привод')
        self.folder.mkdir(parents=True,exist_ok=True)
        with (self.folder/'lio.log').open('ab') as log:self.lio_process=subprocess.Popen(['ros2','launch','mrbobus_lio','l2_lio.launch.py'],stdout=log,stderr=log,start_new_session=True)
    def record(self,enabled):
        with self.operation:
            if enabled:
                if self.recorder and self.recorder.poll() is None:return {'ok':True}
                self.start_lio();self.folder.mkdir(parents=True,exist_ok=True);self.segment+=1
                bag=self.folder/f'bag-{self.segment:03d}'
                topics=['/unilidar/cloud','/unilidar/imu','/lio/odometry','/odom','/tf','/tf_static','/diff_drive_controller/cmd_vel','/camera/image/compressed','/camera/camera_info']
                with (self.folder/'record.log').open('ab') as log:self.recorder=subprocess.Popen(['ros2','bag','record','-o',str(bag),*topics],stdout=log,stderr=log,start_new_session=True)
                time.sleep(.3)
                if self.recorder.poll() is not None:raise ValueError('Запись не запустилась; см. record.log')
            elif self.recorder:
                self.end_process(self.recorder);self.recorder=None
        return {'ok':True}
    @staticmethod
    def end_process(p):
        if p and p.poll() is None:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    def mark(self,label):
        if not isinstance(label,str) or not 1<=len(label.strip())<=100:raise ValueError('Название: 1–100 символов')
        with self.lock:
            if not self.pose or time.monotonic()-self.pose_time>.5:raise ValueError('Нет свежей позы')
            mark={'id':uuid.uuid4().hex[:12],'label':label.strip(),'stamp':self.get_clock().now().nanoseconds/1e9,'pose':dict(self.pose),'source':self.pose_source,'kind':'viewpoint','segment':self.segment};jpeg=self.jpeg;camera_age=time.monotonic()-self.camera_time
            self.folder.mkdir(parents=True,exist_ok=True)
            if jpeg and camera_age<1:
                mark['image']=mark['id']+'.jpg';(self.folder/mark['image']).write_bytes(jpeg)
            with (self.folder/'landmarks.jsonl').open('a') as f:f.write(json.dumps(mark,ensure_ascii=False)+'\n')
            self.marks.append(mark)
        return {'ok':True,'mark':mark}
    def map_action(self,action,data):
        if action=='navigate':return self.navigation.navigate(data)
        if action=='goal':return self.field_map.goal(data)
        if action=='remove':return self.field_map.remove(data)
        with self.gate.lock:
            if self.gate.active:raise ValueError('Для установки позы/совмещения нажми STOP')
            with self.lock:
                if self.pose_source!='LIO' or time.monotonic()-self.pose_time>.5:raise ValueError('Нет свежей LIO-позы')
                current=self.pose_matrix.copy();cloud=list(self.points)
            if action=='pose':return self.field_map.set_pose(data,current)
            if action=='refine':return self.field_map.refine(cloud,current)
        raise ValueError('Неизвестное действие карты')
    def status(self):
        now=time.monotonic()
        try:ready=Path('/run/mrbobus-stop/enabled').read_text().strip()=='1'
        except OSError:ready=False
        with self.gate.lock,self.lock:
            active=self.gate.active and ready and len(self.axes)==4 and all(x['state']==8 and x['error']==0 and now-x['at']<.5 for x in self.axes.values())
            return {'navigation':{'active':self.navigation.active,'message':self.navigation.message,'path':self.navigation.path},'map':self.field_map.status(self.pose_matrix,self.pose_source=='LIO' and now-self.pose_time<.5),'iq':{k:{**v,'age':now-v['at']} for k,v in self.iq.items()},'current_limit':20,'active':active,'ready':ready,'owner':self.gate.owner,'pose':self.pose,'pose_source':self.pose_source,'pose_age':now-self.pose_time if self.pose_time else None,'lio_age':now-self.lio_time if self.lio_time else None,'cloud_age':now-self.cloud_time if self.cloud_time else None,'cloud_hz':round(self.cloud_hz,1),'camera_age':now-self.camera_time if self.camera_time else None,'camera_error':self.camera_error,'axes':{k:{**v,'age':now-v['at']} for k,v in self.axes.items()},'voltage':{k:{'value':v[0],'age':now-v[1]} for k,v in self.voltage.items()},'recording':bool(self.recorder and self.recorder.poll() is None),'session':self.session,'segment':self.segment,'marks':self.marks,'trace':self.trace[-1200:]}
    def camera(self):
        command=['ffmpeg','-nostdin','-loglevel','error','-f','v4l2','-input_format','mjpeg','-video_size','640x480','-framerate','30','-i',self.args.camera,'-vf','fps=5,hflip','-c:v','mjpeg','-q:v','3','-threads','1','-f','image2pipe','pipe:1']
        while self.running:
            try:
                self.camera_process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,start_new_session=True);buffer=b'';last=0.
                while self.running:
                    chunk=self.camera_process.stdout.read(16384)
                    if not chunk:break
                    buffer+=chunk
                    while b'\xff\xd9' in buffer:
                        end=buffer.index(b'\xff\xd9')+2;start=buffer.find(b'\xff\xd8');jpeg=buffer[start:end] if start>=0 else None;buffer=buffer[end:]
                        if jpeg and time.monotonic()-last>=.19:
                            last=time.monotonic()
                            with self.lock:self.jpeg=jpeg;self.camera_time=last;self.camera_error=None
                            msg=CompressedImage();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='camera_optical_frame';msg.format='jpeg';msg.data=jpeg;self.camera_pub.publish(msg)
                    if len(buffer)>4_000_000:buffer=b''
                self.camera_error='Камера недоступна';self.end_process(self.camera_process)
            except Exception as e:self.camera_error=str(e)[:100]
            time.sleep(2)
    def can_listener(self):
        try:
            s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);s.bind(('can0',));s.settimeout(.5)
            while self.running:
                try:raw=s.recv(16)
                except socket.timeout:continue
                ident,length,data=struct.unpack('=IB3x8s',raw)
                if ident&0xe0000000 or length!=8:continue
                n,cmd=ident>>5,ident&31
                if n not in range(4):continue
                with self.lock:
                    if cmd==1:self.axes[str(n)]={'state':data[4],'error':struct.unpack_from('<I',data)[0],'at':time.monotonic()}
                    elif cmd==20:self.iq[str(n)]={'target':struct.unpack_from('<f',data)[0],'measured':struct.unpack_from('<f',data,4)[0],'at':time.monotonic()}
                    elif cmd==23:self.voltage[str(n)]=(struct.unpack_from('<f',data)[0],time.monotonic())
            s.close()
        except OSError:pass
    def close(self):
        self.running=False
        try:self.stop()
        except Exception:pass
        for p in (self.recorder,self.lio_process,self.camera_process):self.end_process(p)

def main():
    p=argparse.ArgumentParser();p.add_argument('--address',default='192.168.67.149');p.add_argument('--port',type=int,default=8080);p.add_argument('--camera',default='/dev/video0');p.add_argument('--records',default='/home/taras/robot/records');p.add_argument('--web',default=str(ROOT/'src/mrbobus_console/web'));args=p.parse_args()
    rclpy.init();node=Console(args);executor=SingleThreadedExecutor();executor.add_node(node);spin_thread=threading.Thread(target=executor.spin,daemon=True);spin_thread.start();node.start_lio();origin=f'http://{args.address}:{args.port}'
    class Handler(BaseHTTPRequestHandler):
        protocol_version='HTTP/1.1'
        def log_message(self,*args):pass
        def reply(self,code,data,content='application/json'):
            data=data if isinstance(data,bytes) else json.dumps(data,allow_nan=False).encode();self.send_response(code);self.send_header('Content-Type',content);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=self.path.split('?')[0]
            if path in ('/','/app.js','/map.js','/style.css'):
                name={'/':'index.html','/app.js':'app.js','/map.js':'map.js','/style.css':'style.css'}[path];content={'/':'text/html; charset=utf-8','/app.js':'text/javascript','/map.js':'text/javascript','/style.css':'text/css'}[path];self.reply(200,(Path(args.web)/name).read_bytes(),content)
            elif path=='/api/map':self.reply(200,node.field_map.data) if node.field_map.data else self.reply(404,{'error':'Нет карты'})
            elif path=='/api/status':self.reply(200,node.status())
            elif path=='/api/cloud':
                with node.lock:result={'points':node.points[::3] if node.gate.active else node.points,'pose':node.pose,'source':node.pose_source,'stats':node.cloud_stats,'age':time.monotonic()-node.cloud_time if node.cloud_time else None}
                self.reply(200,result)
            elif path=='/camera.jpg':
                with node.lock:image=node.jpeg
                self.reply(200,image,'image/jpeg') if image else self.reply(503,{'error':'Камера ещё не готова'})
            else:self.reply(404,{'error':'Not found'})
        def do_POST(self):
            if self.headers.get('Origin')!=origin:self.reply(403,{'error':'Wrong origin'});return
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('Invalid request')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict):raise ValueError('JSON object required')
                if self.path.startswith('/api/map/') :result=node.map_action(self.path.rsplit('/',1)[-1],data)
                elif self.path=='/api/stop':result=node.stop()
                elif self.path=='/api/arm':result=node.arm(data.get('client'))
                elif self.path=='/api/cmd':result=node.command(data)
                elif self.path=='/api/record':result=node.record(data.get('enabled') is True)
                elif self.path=='/api/mark':result=node.mark(data.get('label'))
                else:self.reply(404,{'error':'Not found'});return
                self.reply(200,result)
            except Exception as e:self.reply(409,{'error':str(e)[:250]})
    class Server(ThreadingHTTPServer):
        daemon_threads=True
        def get_request(self):
            sock,addr=super().get_request();sock.settimeout(3);return sock,addr
    server=Server((args.address,args.port),Handler)
    def shutdown(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown)
    try:server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:pass
    finally:server.server_close();node.close();executor.shutdown(timeout_sec=2);spin_thread.join(timeout=2);node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
