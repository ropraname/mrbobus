"""Render capture_nav_obstacles JSON without treating inflation as a real object."""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('--output',required=True);a=p.parse_args();s=json.loads(Path(a.input).read_text())
fig,axes=plt.subplots(1,2,figsize=(13,6));colors=ListedColormap(['#959595','#ffffff','#ffedd0','#ffc978','#ee9148','#e44938','#6f0a0a']);norm=BoundaryNorm([-1.5,-.5,.5,25.5,50.5,75.5,98.5,100.5],colors.N)
for ax,key in zip(axes,['global','local']):
 l=s['layers'].get(key)
 if not l:ax.set_title(key+' unavailable');continue
 grid=np.array(l['data']).reshape(l['height'],l['width']);x,y,_=l['origin']['position'];r=l['resolution'];extent=[x,x+l['width']*r,y,y+l['height']*r]
 ax.imshow(grid,origin='lower',extent=extent,cmap=colors,norm=norm,interpolation='nearest')
 robot=l.get('robot')
 if robot:
  px,py,_=robot['translation'];qx,qy,qz,qw=robot['rotation'];yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
  rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
  for dx,dy,style,label in [(.15351,.1735,'-','body'),(.19351,.2135,'--','body + 4cm')]:
   v=np.array([[-dx,-dy],[dx,-dy],[dx,dy],[-dx,dy],[-dx,-dy]])@rot.T+[px,py];ax.plot(v[:,0],v[:,1],style,color='#0450db',lw=2,label=label)
  ax.arrow(px,py,.3*math.cos(yaw),.3*math.sin(yaw),width=.015,color='#0450db')
  if key=='local':ax.set_xlim(px-1.4,px+1.4);ax.set_ylim(py-1.4,py+1.4)
 path=s['layers'].get('plan',{})
 if path.get('frame')==l['frame'] and path.get('path'):
  xy=np.array(path['path']);ax.plot(xy[:,0],xy[:,1],color='#00a479',label='Nav2 path')
 ax.set_title(f"{key} / {l['frame']} / received age {l['received_age']:.2f}s")
 ax.set_aspect('equal');ax.set_xlabel('m');ax.set_ylabel('m');ax.legend(loc='upper right',fontsize=8)
fig.suptitle(s['time']+'\n'+s['status'].get('navigation',{}).get('message',''))
fig.text(.5,.01,'White: free | orange: graded cost / inflation | red: high cost | dark red: occupied | grey: unknown',ha='center')
fig.tight_layout(rect=[0,.035,1,.91]);fig.savefig(a.output,dpi=150)
