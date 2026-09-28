#!/usr/bin/env python3
"""Read legacy ODrive ASCII settings; optionally raise only torque_lim in RAM.

Example: --port /dev/ttyACM0 --apply-current-limit 15
Never arms motors, calibrates, saves flash, or reboots. Both axes must be Idle.
"""
import argparse
import datetime
import json
import math
import os
from pathlib import Path
import select
import termios
import time
import tty

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True)
target_group=p.add_mutually_exclusive_group()
target_group.add_argument('--apply-current-limit',type=float,default=None)
target_group.add_argument('--torque-limit',type=float,default=None,help='Explicit torque ceiling in Nm (up to 2); current limit is separate')
p.add_argument('--backup-dir',default='.local/odrive-backups')
a=p.parse_args()
if a.apply_current_limit is not None and not 1<=a.apply_current_limit<=15:
    p.error('Requested equivalent current must be 1–15 A')
if a.torque_limit is not None and not 0<a.torque_limit<=2:
    p.error('Torque limit must be in (0,2] Nm')
fd=os.open(a.port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
original=termios.tcgetattr(fd)
def write(command):os.write(fd,(command+'\n').encode('ascii'))
def read(field):
    termios.tcflush(fd,termios.TCIFLUSH)
    write('r '+field)
    data=b'';end=time.monotonic()+1.5
    while time.monotonic()<end:
        ready,_,_=select.select([fd],[],[],max(0,end-time.monotonic()))
        if ready:
            data+=os.read(fd,4096)
            if b'\n' in data:return data.split(b'\n',1)[0].decode('ascii').strip()
    raise RuntimeError('No ASCII reply for '+field)
def number(field):
    value=float(read(field))
    if not math.isfinite(value) and not (field.endswith('.motor.config.torque_lim') and value==math.inf):
        raise RuntimeError('Non-finite '+field)
    return value
try:
    tty.setraw(fd)
    version=[int(number('fw_version_'+k)) for k in ('major','minor','revision')]
    if version!=[0,5,6]:raise RuntimeError('Expected fw0.5.6, got '+str(version))
    serial=read('serial_number')
    before={}
    for axis in ('axis0','axis1'):
        before[axis]={key:number(axis+'.'+key) for key in (
            'current_state','config.can.node_id','motor.config.torque_lim',
            'motor.config.torque_constant','motor.config.current_lim',
            'controller.config.vel_gain','controller.config.vel_integrator_gain','encoder.config.bandwidth')}
    if {int(v['config.can.node_id']) for v in before.values()} not in ({0,1},{2,3}):
        raise RuntimeError('Unexpected CAN node IDs; refusing changes')
    if any(v['current_state']!=1 for v in before.values()):
        raise RuntimeError('Both axes must be Idle')
    print(json.dumps({'serial':serial,'firmware':version,'before':before},indent=2))
    if a.apply_current_limit is not None or a.torque_limit is not None:
        targets={axis:a.torque_limit if a.torque_limit is not None else values['motor.config.torque_constant']*a.apply_current_limit for axis,values in before.items()}
        if any(not .001<t<=5 for t in targets.values()):raise RuntimeError('Unexpected torque constant/target')
        directory=Path(a.backup_dir);directory.mkdir(parents=True,exist_ok=True)
        backup=directory/(datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+serial+'.json')
        backup.write_text(json.dumps({'serial':serial,'port':a.port,'before':before,'requested_torque_limits':targets},indent=2))
        changed=[]
        try:
            for axis,target in targets.items():
                if any(number(x+'.current_state')!=1 for x in before):raise RuntimeError('Axis left Idle')
                changed.append(axis)
                write(f'w {axis}.motor.config.torque_lim {target:.9g}')
                time.sleep(.05)
                actual=number(axis+'.motor.config.torque_lim')
                if abs(actual-target)>max(1e-5,target*1e-4):raise RuntimeError('Readback mismatch on '+axis)
        except Exception:
            for axis in changed:
                if number(axis+'.current_state')==1:
                    write(f"w {axis}.motor.config.torque_lim {before[axis]['motor.config.torque_lim']:.9g}")
            raise
        print(json.dumps({'applied_in_ram':targets,'backup':str(backup),'flash_saved':False},indent=2))
finally:
    termios.tcsetattr(fd,termios.TCSANOW,original)
    os.close(fd)
