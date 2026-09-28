#!/usr/bin/env python3
"""One-command bounded Pi trial; never clears the user's latched STOP.

Run on Mac: python3 tools/run_drive_trial.py --profile low_i --distance .25
Use --reverse only along a physically clear path. Results stay in ignored .local.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import shlex
import shutil
import re
import subprocess
import tarfile
import io

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--profile', default='baseline', help='Named entry in config/tuning.yaml')
p.add_argument('--current-limit', type=float, default=3.)
p.add_argument('--arc-pair', action='store_true')
p.add_argument('--turn-pair', action='store_true')
p.add_argument('--speed', type=float, default=.03)
p.add_argument('--distance', type=float, default=.25)
p.add_argument('--reverse', action='store_true')
p.add_argument('--check', action='store_true', help='Sync and check readiness; never activate motors')
p.add_argument('--host', default='taras@mrbobus.local')
p.add_argument('--ssh-control', default='/tmp/mrbobus-audit/ssh-control')
a=p.parse_args()
if not math.isfinite(a.current_limit) or not 1<=a.current_limit<=15:p.error('Current limit must be 1–15 A')
if sum([a.turn_pair,a.arc_pair,a.reverse])>1:p.error('Choose one motion mode')
if not re.fullmatch(r'[a-z][a-z0-9_]{0,31}',a.profile):p.error('Invalid profile name')
if not math.isfinite(a.speed) or not 0<a.speed<=.06:p.error('Speed must be in (0,0.06] m/s')
if not math.isfinite(a.distance) or not 0<a.distance<=.5:p.error('Distance must be in (0,0.5] m')
root=Path(__file__).resolve().parent.parent
ssh=['ssh','-S',a.ssh_control,a.host]
remote='/home/taras/robot/mrbobus_ws'
# Config/scripts only; C++ changes are built/tested separately, never implicitly.
files=['tools/read_bus.py','tools/check_drive.py','src/mrbobus_bringup/config/tuning.yaml','src/mrbobus_bringup/config/floor.yaml']
buffer=io.BytesIO()
with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
    for path in files:archive.add(root/path,arcname=path)
subprocess.run(ssh+['tar -xzf - -C '+shlex.quote(remote)],input=buffer.getvalue(),check=True)
if a.check:
    subprocess.run(ssh+["systemctl is-active mrbobus-stop; printf 'STOP flag: '; cat /run/mrbobus-stop/enabled; test -f /home/taras/robot/mrbobus_ws/install/mrbobus_bringup/share/mrbobus_bringup/config/tuning.yaml"],check=True)
    print('Preparation check complete. No motor activation.')
    raise SystemExit(0)
trial=datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+a.profile+('-arc-pair' if a.arc_pair else '-turn-pair' if a.turn_pair else '-reverse' if a.reverse else '-forward')
out=root/'.local'/'trials'/trial;out.mkdir(parents=True)
shutil.copy2(root/'src/mrbobus_bringup/config/tuning.yaml',out/'tuning.yaml')
shutil.copy2(root/'src/mrbobus_bringup/config/floor.yaml',out/'floor.yaml')
remote_out=remote+'/log/trials/'+trial
script='''set -euo pipefail
profile=$1; distance=$2; direction=$3; output=$4; speed=$5; current=$6
if [[ $(cat /run/mrbobus-stop/enabled) != 1 ]]; then
  echo 'STOP is latched. Trial refused; unblock explicitly in the browser.' >&2; exit 73
fi
mkdir -p "$output"
printf 'DRIVE_GAIN_PROFILE=%s\\nDRIVE_CURRENT_LIMIT=%s\\n' "$profile" "$current" > /run/mrbobus-stop/control.env.new
mv /run/mrbobus-stop/control.env.new /run/mrbobus-stop/control.env
# Stop the drive service whenever this bounded trial shell exits.
trap 'sudo systemctl stop mrbobus-control' EXIT
trap 'exit 130' HUP INT TERM
sudo systemctl stop mrbobus-control robot-base
sudo install -d -o taras -g taras /run/robot-base
python3 /home/taras/robot/mrbobus_ws/tools/read_bus.py
sudo systemctl start mrbobus-control
set +u
source /opt/ros/jazzy/setup.bash
source /home/taras/robot/mrbobus_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/taras/robot/ros2_ws/deploy/cyclone-pi.xml
extra=(); [[ "$direction" == reverse ]] && extra+=(--reverse)
[[ "$direction" == turn ]] && extra+=(--turn-pair)
[[ "$direction" == arc ]] && extra+=(--arc-pair)
python3 /home/taras/robot/mrbobus_ws/tools/check_drive.py --floor --distance "$distance" --speed "$speed" "${extra[@]}" --output "$output/samples.json"
'''
args=['bash','-s','--',a.profile,str(a.distance),'arc' if a.arc_pair else 'turn' if a.turn_pair else 'reverse' if a.reverse else 'forward',remote_out,str(a.speed),str(a.current_limit)]
motion='20 deg arcs left and right' if a.arc_pair else '30 deg left and return' if a.turn_pair else f'{a.distance:g} m at {a.speed:g} m/s'
print(f'Trial {trial}: {motion}, {a.current_limit:g} A. STOP remains available.',flush=True)
result=subprocess.run(ssh+[shlex.join(args)],input=script,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(out/'run.log').write_text(result.stdout)
if result.returncode: print(result.stdout)
subprocess.run(['scp','-q','-r','-o','ControlPath='+a.ssh_control,a.host+':'+remote_out+'/.',str(out)],check=False)
(out/'trial.json').write_text(json.dumps({'profile':a.profile,'current_limit':a.current_limit,'distance':a.distance,'speed':a.speed,'reverse':a.reverse,'turn_pair':a.turn_pair,'arc_pair':a.arc_pair,'exit_code':result.returncode},indent=2))
summary_path=out/'samples.summary.json'
if summary_path.exists():
    summary=json.loads(summary_path.read_text())
    print(json.dumps({'result':result.returncode,'odom_x':summary.get('odom_x'),'max_abs_yaw_deg':summary.get('max_abs_yaw_deg'),'peak_average_rpm':summary.get('peak_average_rpm')},indent=2))
print('Saved:',out)
raise SystemExit(result.returncode)
