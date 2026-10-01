"""Publication view of measured point cloud, never a generated scene."""
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap,Normalize
p=argparse.ArgumentParser();p.add_argument('cloud');p.add_argument('output');args=p.parse_args()
a=np.load(args.cloud);points=a['points'];track=a['trajectory']
# Same enclosure crop as the reference map; suppress high boundary returns
# (spectators/outside scene), without modifying the recorded input file.
x,y,z=points.T
bounds=(x>=-2.056)&(x<=2.122)&(y>=-1.947)&(y<=2.193)&(z>=-.08)&(z<=1.5)
edge=(x<-1.876)|(x>1.942)|(y<-1.767)|(y>2.013)
points=points[bounds&~(edge&(z>.60))]
fig=plt.figure(figsize=(14,10),facecolor='#0b111c');ax=fig.add_axes([0,.07,1,.83],projection='3d',facecolor='#0b111c')
cmap=LinearSegmentedColormap.from_list('height',['#3d6572','#78c7c2','#dfd7a3','#ff704d'])
ax.scatter(*points.T,c=points[:,2],cmap=cmap,norm=Normalize(0,1.1),s=1.7,alpha=.88,linewidths=0,depthshade=False,rasterized=True)
ax.plot(track[:,0],track[:,1],track[:,2]+.025,color='#4bffe0',lw=1.3,alpha=.85)
ax.set_xlim(-2.2,2.25);ax.set_ylim(-2.1,2.35);ax.set_zlim(-.12,1.5);ax.set_box_aspect((4.45,4.45,1.62));ax.view_init(elev=38,azim=-52);ax.set_axis_off()
fig.text(.055,.925,'БОБУС / 3D-КАРТА ПОЛИГОНА',color='white',size=24,weight='bold')
fig.text(.055,.887,'Unitree L2 + Point-LIO   •   реальные измерения, 28.09.2026',color='#91a9bd',size=13)
fig.text(.055,.055,'Цвет — высота точек     /     Бирюзовая линия — траектория записи',color='#b1c4d3',size=12)
fig.text(.055,.028,'Облако обрезано по границе поля; высокие отражения у борта скрыты для обзора.',color='#748d9f',size=10)
fig.savefig(args.output,dpi=140,facecolor=fig.get_facecolor())
