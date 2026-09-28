#!/usr/bin/env python3
"""Read bus voltage only while all four axes are Idle and CAN lock is free.

Legacy RTR requests feed watchdogs; never run alongside a drive owner.
"""
import fcntl
import json
from pathlib import Path
import socket
import struct
import time

lock=Path('/run/robot-base/robot-base-can0.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
s.bind(('can0',));s.settimeout(.05)
axes={}; voltages={}
def read():
    try: frame=s.recv(16)
    except socket.timeout: return
    ident,length,data=struct.unpack('=IB3x8s',frame)
    if ident&0xe0000000 or length!=8:return
    n,cmd=ident>>5,ident&31
    if cmd==1 and n in range(4):axes[n]=(data[4],time.monotonic())
    elif cmd==23 and n in (0,2):voltages[n]=struct.unpack('<ff',data)[0]
try:
    end=time.monotonic()+.4
    while time.monotonic()<end:read()
    if len(axes)!=4 or any(v[0]!=1 for v in axes.values()):
        raise RuntimeError('All axes must be Idle before diagnostic requests')
    for n in (0,2):
        s.send(struct.pack('=IB3x8s',socket.CAN_RTR_FLAG|(n<<5)|23,8,b'\0'*8));time.sleep(.005)
    end=time.monotonic()+.5
    while time.monotonic()<end and len(voltages)<2:read()
    if len(voltages)!=2:raise RuntimeError('No bus voltage response from both boards')
    print(json.dumps({'voltage':voltages,'axes_idle':True}))
    if any(not 33.<=v<=43. for v in voltages.values()):
        raise SystemExit('Outside commissioning voltage window 33–43 V; do not run trial')
finally:s.close();lock.close()
