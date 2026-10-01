#!/usr/bin/env python3
"""Read voltage/errors with Idle axes and exclusive CAN ownership.
Optional explicit reset accepts only the known overspeed error combination.

Legacy RTR requests feed watchdogs; never run alongside a drive owner.
"""
import argparse
import fcntl
import json
from pathlib import Path
import socket
import struct
import time

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--encoder-counts',action='store_true',help='Read raw Hall counters instead of voltage; never enables motors')
parser.add_argument('--errors',action='store_true',help='Read legacy motor/encoder/controller errors, without clearing them')
parser.add_argument('--clear-known-overspeed',action='store_true',help='With --errors only: clear OVERSPEED and its known cascade in Idle; no motor enable')
args=parser.parse_args()
if args.clear_known_overspeed and not args.errors:parser.error('--clear-known-overspeed requires --errors')
lock=Path('/run/robot-base/robot-base-can0.lock').open('a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
s.bind(('can0',));s.settimeout(.05)
axes={}; voltages={}; counts={}; errors={}
def read():
    try: frame=s.recv(16)
    except socket.timeout: return
    ident,length,data=struct.unpack('=IB3x8s',frame)
    if ident&0xe0000000 or length not in (4,8):return
    n,cmd=ident>>5,ident&31
    if cmd==1 and n in range(4):axes[n]=(data[4],time.monotonic())
    elif cmd==23 and n in (0,2):voltages[n]=struct.unpack('<ff',data)[0]
    elif cmd==10 and n in range(4):counts[n]=dict(zip(('shadow_count','count_in_cpr'),struct.unpack('<ii',data)))
    elif cmd in (3,4,29) and n in range(4):errors[f'{n}:{cmd}']=int.from_bytes(data[:8 if cmd==3 else 4],'little')
try:
    end=time.monotonic()+.4
    while time.monotonic()<end:read()
    if len(axes)!=4 or any(v[0]!=1 for v in axes.values()):
        raise RuntimeError('All axes must be Idle before diagnostic requests')
    for n in (range(4) if args.encoder_counts or args.errors else (0,2)):
        for cmd in ((3,4,29) if args.errors else (10,) if args.encoder_counts else (23,)):
            s.send(struct.pack('=IB3x8s',socket.CAN_RTR_FLAG|(n<<5)|cmd,8,b'\0'*8));time.sleep(.005)
    end=time.monotonic()+.5
    while time.monotonic()<end and (len(errors)<12 if args.errors else len(counts)<4 if args.encoder_counts else len(voltages)<2):read()
    if args.errors:
        print(json.dumps({'errors':errors,'axes_idle':True}))
        if len(errors)!=12:raise RuntimeError('Missing error responses')
        if args.clear_known_overspeed:
            if Path('/run/mrbobus-stop/enabled').read_text().strip()!='0':raise RuntimeError('STOP required')
            if any(errors[f'{n}:4'] or errors[f'{n}:29'] not in (0,1) or
                   errors[f'{n}:3'] & ~0x110000000 for n in range(4)):
                raise RuntimeError('Unrecognised fault: refusing automatic reset')
            for n in range(4):
                if any(state!=1 or time.monotonic()-at>.5 for state,at in axes.values()):raise RuntimeError('Axes must remain fresh and Idle')
                s.send(struct.pack('=IB3x8s',(n<<5)|24,8,b'\0'*8));time.sleep(.005)
            print(json.dumps({'cleared_known_overspeed':True,'motor_enable_sent':False}))
        raise SystemExit(0)
    if args.encoder_counts:
        if len(counts)!=4:raise RuntimeError('Missing encoder count response')
        print(json.dumps({'encoder_counts':counts,'axes_idle':True}))
        raise SystemExit(0)
    if len(voltages)!=2:raise RuntimeError('No bus voltage response from both boards')
    print(json.dumps({'voltage':voltages,'axes_idle':True}))
    if any(not 33.<=v<=43. for v in voltages.values()):
        raise SystemExit('Outside commissioning voltage window 33–43 V; do not run trial')
finally:s.close();lock.close()
