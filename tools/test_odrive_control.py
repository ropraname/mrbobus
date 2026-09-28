"""End-to-end official-plugin adaptation + diff_drive_controller test on vcan0.

Requires built mrbobus_bringup/odrive_ros2_control and Linux vcan0.
Starts its own controller_manager in domain43. Never opens physical CAN.
"""
import json
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '43':
    raise SystemExit('Test requires ROS_DOMAIN_ID=43')
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers, SwitchController

fault_kind=sys.argv[1] if len(sys.argv)>1 else 'stale'
stop_path=Path('/tmp/mrbobus-vcan-stop-flag')
if fault_kind=='stop':stop_path.write_text('1')

sock=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
sock.bind(('vcan0',));sock.settimeout(.002)
state={n:1 for n in range(4)}; error={n:2048 for n in state}
gains={};limits={}
test_current=float(os.environ.get("MRBOBUS_TEST_CURRENT_LIMIT","1.0"))
velocity={n:0. for n in state};position=dict(velocity)
last_rx={n:time.monotonic() for n in state}; silent=set()
peak={n:0. for n in state}; counts={n:0 for n in state}
stop=threading.Event()
def packet(n,cmd,data):return struct.pack('=IB3x8s',(n<<5)|cmd,len(data),data.ljust(8,b'\0'))
def simulate():
    before=next_send=time.monotonic()
    while not stop.is_set():
        now=time.monotonic();dt=now-before;before=now
        for n in state:
            if now-last_rx[n]>2.0:state[n]=1;error[n]=2048
            if state[n]==8:position[n]+=velocity[n]*dt
        if now>=next_send:
            for n in state:
                if n in silent:continue
                sock.send(packet(n,1,struct.pack('<IBBBB',error[n],state[n],0,0,0)))
                sock.send(packet(n,9,struct.pack('<ff',position[n] if state[n]==8 else 0.,velocity[n] if state[n]==8 else 0.)))
            next_send=now+.01
        try:raw=sock.recv(16)
        except socket.timeout:continue
        ident,length,data=struct.unpack('=IB3x8s',raw)
        if ident&0xe0000000:continue
        n,cmd=ident>>5,ident&31
        if n not in state:continue
        last_rx[n]=now
        if cmd==7:
            state[n]=struct.unpack_from('<I',data)[0]
            if state[n]==8:position[n]=.2*(n+1)  # Legacy estimator rebases when enabled.
        elif cmd==13:
            velocity[n]=struct.unpack_from('<f',data)[0]
            peak[n]=max(peak[n],abs(velocity[n]));counts[n]+=1
        elif cmd==15:limits[n]=struct.unpack('<ff',data)
        elif cmd==24:error[n]=0
        elif cmd==27:gains[n]=struct.unpack('<ff',data)

thread=threading.Thread(target=simulate,daemon=True);thread.start()
rclpy.init();node=rclpy.create_node('odrive_vcan_test');latest={}
subs=[node.create_subscription(Odometry,'/odom',lambda m:latest.update(odom=m),10),
      node.create_subscription(JointState,'/joint_states',lambda m:latest.update(joints=m),10)]
pub=node.create_publisher(TwistStamped,'/diff_drive_controller/cmd_vel',10)
clients={k:node.create_client(t,'/controller_manager/'+k) for k,t in [('list_controllers',ListControllers),('switch_controller',SwitchController)]}
log_path=Path('/tmp/mrbobus-odrive-vcan.log');log=log_path.open('w')
proc=subprocess.Popen(['ros2','launch','mrbobus_bringup','control.launch.py','can:=vcan0','current_limit:='+str(test_current),'gain_profile:='+('soft' if fault_kind=='soft' else 'existing')] + (['stop_file:='+str(stop_path)] if fault_kind=='stop' else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def wait_for(predicate,timeout=10.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if predicate():return
    raise AssertionError('Condition timeout; see '+str(log_path))
def service(name,request):
    client=clients[name]
    assert client.wait_for_service(timeout_sec=15.),name
    future=client.call_async(request)
    wait_for(future.done,15.)
    return future.result()
def controller_ready():
    result=service('list_controllers',ListControllers.Request())
    return any(c.name=='diff_drive_controller' and c.state=='inactive' for c in result.controller)
def drive(v,w,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_link'
        m.twist.linear.x=v;m.twist.angular.z=w;pub.publish(m)
        rclpy.spin_once(node,timeout_sec=.02);time.sleep(.02)
try:
    wait_for(controller_ready,20.)
    assert all(s==1 for s in state.values()),'Startup must remain Idle'
    request=SwitchController.Request();request.activate_controllers=['diff_drive_controller'];request.strictness=2
    assert service('switch_controller',request).ok
    wait_for(lambda:all(s==8 for s in state.values()))
    if fault_kind=='soft':
        assert all(abs(gains[n][0]-(.05 if n<2 else .0265))<1e-6 for n in range(4)),gains
    drive(.01,0.,3.)
    assert len(limits)==4 and all(abs(v[1]-test_current)<1e-5 for v in limits.values()),limits
    wait_for(lambda:'odom' in latest and latest['odom'].pose.pose.position.x>.01)
    assert velocity[0]>0 and velocity[3]>0 and velocity[1]<0 and velocity[2]<0,velocity
    assert max(peak.values()) <= .1+1e-5,peak
    measured=latest['odom'].pose.pose.position.x
    before=dict(counts);drive(.01,0.,.5)
    assert all(counts[n]-before[n]>15 for n in state), 'Setpoint not repeated every cycle'
    start=time.monotonic()
    wait_for(lambda:all(abs(v)<1e-5 for v in velocity.values()),2.)
    timeout=time.monotonic()-start
    # Lifecycle deactivation must send Idle to every drive; explicit reactivation is required.
    disable=SwitchController.Request();disable.deactivate_controllers=['diff_drive_controller'];disable.strictness=2
    assert service('switch_controller',disable).ok
    wait_for(lambda:all(s==1 for s in state.values()))
    time.sleep(.15)
    assert service('switch_controller',request).ok
    wait_for(lambda:all(s==8 for s in state.values()))
    drive(.01,0.,2.)
    if fault_kind=='error':error[2]=64;state[2]=1
    elif fault_kind=='stop':stop_path.write_text('0')
    else:silent.add(2)
    drive(.01,0.,.9)
    wait_for(lambda:all(s==1 for s in state.values()),2.)
    drive(.01,0.,.4)
    assert all(s==1 for s in state.values()),'Fault must not auto-rearm'
    print(json.dumps({'result':'PASS','forward_m':measured,'timeout_stop_s':timeout,
                      'peak_motor_rps':peak,'fault_stops_all':fault_kind,'lifecycle_idle':True,'startup_idle':True},indent=2))
finally:
    if proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close();stop.set();thread.join();sock.close();node.destroy_node();rclpy.shutdown()
