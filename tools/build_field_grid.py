"""Conservative grid from observed floor and physically traversed strip."""
import argparse
import numpy as np
from PIL import Image,ImageDraw,ImageFilter
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('export_directory',type=Path);p=parser.parse_args().export_directory;a=np.load(p/'field.npz');points=a['points'];lo,hi=a['bounds'];res=.05;wh=np.ceil((hi-lo)/res).astype(int)+1
# Unknown is retained outside observed floor and the physically traversed strip.
free=Image.new('L',tuple(wh),0);d=ImageDraw.Draw(free)
for q in points[(points[:,2]>-.04)&(points[:,2]<.06)]:
 x,y=((q[:2]-lo)/res).astype(int);d.point((int(x),int(y)),fill=255)
free=free.filter(ImageFilter.MaxFilter(3));d=ImageDraw.Draw(free)
line=[tuple(((q[:2]-lo)/res).astype(int)) for q in a['trajectory']];d.line(line,fill=255,width=9)
obs=Image.new('L',tuple(wh),0);d=ImageDraw.Draw(obs)
for q in points[(points[:,2]>.10)&(points[:,2]<1.25)]:
 x,y=((q[:2]-lo)/res).astype(int);d.point((int(x),int(y)),fill=255)
grid=np.full((wh[1],wh[0]),205,np.uint8);grid[np.asarray(free)>0]=254;grid[np.asarray(obs)>0]=0
confirmed=Image.new('L',tuple(wh),0);ImageDraw.Draw(confirmed).line(line,fill=255,width=9);grid[np.asarray(confirmed)>0]=254
Image.fromarray(np.flipud(grid)).save(p/'field.pgm');(p/'field.yaml').write_text(f'image: field.pgm\nmode: trinary\nresolution: {res}\norigin: [{lo[0]}, {lo[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
print('grid',wh,'free',sum((grid==254).flat),'occupied',sum((grid==0).flat))
