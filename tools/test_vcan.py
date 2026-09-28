"""Integration test on a VIRTUAL interface only. Never opens can0."""
import math
import socket
import struct
import sys
import threading
import time
from robot_base.core import Base, SIGNS, frame

interface = sys.argv[1] if len(sys.argv)>1 else 'vcan0'
assert interface.startswith('vcan'), 'Only virtual CAN permitted'
host = socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
fake = socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
host.bind((interface,));fake.bind((interface,))
host.setblocking(False);fake.settimeout(.003)
stop=threading.Event()
states={n:1 for n in SIGNS};vel={n:0. for n in SIGNS};position={n:0. for n in SIGNS}
last={n:time.monotonic() for n in SIGNS}
silent=set()
def simulator():
    old=next_hb=time.monotonic()
    while not stop.is_set():
        now=time.monotonic();dt=now-old;old=now
        for n in SIGNS:
            if now-last[n]>2:states[n]=1;vel[n]=0
            position[n]+=vel[n]*dt if states[n]==8 else 0
        if now>=next_hb:
            for n in SIGNS:
                if n not in silent:fake.send(frame(n,1,struct.pack('<IBBBB',0,states[n],0,0,0)))
            next_hb=now+.05
        try:raw=fake.recv(16)
        except socket.timeout:continue
        ident,length,data=struct.unpack('=IB3x8s',raw);n=(ident&0x7ff)>>5;cmd=ident&31
        if n not in SIGNS:continue
        last[n]=now
        if cmd==7:states[n]=struct.unpack_from('<I',data)[0]
        elif cmd==13:vel[n]=struct.unpack_from('<f',data)[0]
        elif cmd==10 and n not in silent:
            count=round(position[n]*72)
            fake.send(frame(n,10,struct.pack('<ii',count,count%72)))

thread=threading.Thread(target=simulator);thread.start()
b=Base(host.send,time.monotonic,True)
seq=0
def run(seconds, commands=False):
    global seq
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        cycle=time.monotonic()
        while True:
            try:b.receive(host.recv(16),time.monotonic())
            except BlockingIOError:break
        if commands:
            seq+=1;b.command('test',seq,.04,0,time.monotonic())
        b.tick(time.monotonic())
        time.sleep(max(0,.05-(time.monotonic()-cycle)))
try:
    run(.6);assert b.arm('test',time.monotonic())[0]
    run(2,True);assert b.state=='RUNNING' and b.x>0
    run(1.6);assert b.state=='IDLE' and all(s==1 for s in states.values())
    print('PASS: codec, arming, forward odometry, command expiry and controlled stop')
    run(.2);assert b.arm('test',time.monotonic())[0]
    run(.6,True);silent.add(2);run(.7,True)
    assert b.state=='FAULT';run(.6)
    assert all(s==1 for s in states.values())
    print('PASS: missing feedback stops all axes and latches fault')
    # Independent simulated drive watchdog with the controller gone.
    for n in SIGNS:
        host.send(frame(n,7,struct.pack('<I',8)))
        host.send(frame(n,13,struct.pack('<ff',.1,0)))
    time.sleep(2.3)
    assert all(s==1 for s in states.values())
    print('PASS: independent 2 s watchdog after command process stops')
finally:
    stop.set();thread.join();host.close();fake.close()
