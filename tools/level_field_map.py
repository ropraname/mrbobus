"""Offline floor alignment and field preview; preserves original LIO map."""
import argparse
import numpy as np,json
from pathlib import Path
from PIL import Image,ImageDraw
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('export_directory',type=Path);p=parser.parse_args().export_directory;c=np.load(p/'map.npz')['points'];t=np.load(p/'trajectory.npz');b=t['base'];pos=b[:,:3,3];ts=t['time']-t['time'][0]
# First estimate tilt from traversed surface, then fit observed floor points.
A=np.c_[pos[:,:2],np.ones(len(pos))];coef=np.linalg.lstsq(A,pos[:,2]-.045,rcond=None)[0]
mask=(c[:,0]>pos[:,0].min()-.3)&(c[:,0]<pos[:,0].max()+.3)&(c[:,1]>pos[:,1].min()-.3)&(c[:,1]<pos[:,1].max()+.3)&(abs(c[:,2]-(np.c_[c[:,:2],np.ones(len(c))]@coef))<.18)
sample=c[mask];rng=np.random.default_rng(7);sample=sample[rng.choice(len(sample),min(16000,len(sample)),replace=False)];best=np.zeros(len(sample),bool)
for _ in range(200):
 q=sample[rng.choice(len(sample),3,replace=False)];n=np.cross(q[1]-q[0],q[2]-q[0]);n/=max(np.linalg.norm(n),1e-12)
 if abs(n[2])<.94:continue
 inliers=abs((sample-q[0])@n)<.018
 if inliers.sum()>best.sum():best=inliers
floor=sample[best];center=floor.mean(0);_,_,vh=np.linalg.svd(floor-center,full_matrices=False);n=vh[-1];n*=np.sign(n[2]);v=np.cross(n,[0,0,1]);sk=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]]);R=np.eye(3)+sk+sk@sk/(1+n[2]);offset=-R@center;M=np.eye(4);M[:3,:3]=R;M[:3,3]=offset
out=c@R.T+offset;base=np.einsum('ij,njk->nik',M,b);path=base[:,:3,3]
np.savez_compressed(p/'map-level.npz',points=out);np.savez_compressed(p/'trajectory-level.npz',time=t['time'],base=base)
with (p/'map-level.ply').open('wb') as f:
 f.write(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(out)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode());f.write(out.astype('<f4').tobytes())
summary={'floor_normal_lio':n.tolist(),'tilt_deg':float(np.rad2deg(np.arccos(n[2]))),'floor_inliers':int(best.sum()),'floor_candidates':len(sample),'floor_residual_p95_m':float(np.percentile(abs((floor-center)@n),95)),'base_height_percentiles_m':np.percentile(path[:,2],[0,5,50,95,100]).tolist(),'map_from_lio':M.tolist()};(p/'level-summary.json').write_text(json.dumps(summary,indent=2));print(summary)
# Local field view, excluding the exhibition hall and overhead surfaces.
lo=path[:,:2].min(0)-.45;hi=path[:,:2].max(0)+.45;scale=180
W,H=int((hi[0]-lo[0])*scale),int((hi[1]-lo[1])*scale);im=Image.new('RGB',(W+60,H+150),'#111b26');d=ImageDraw.Draw(im)
def xy(q):return np.stack([30+(q[:,0]-lo[0])*scale,30+(hi[1]-q[:,1])*scale],1).astype(int)
sel=(out[:,2]>.035)&(out[:,2]<1.4)&(out[:,0]>lo[0])&(out[:,0]<hi[0])&(out[:,1]>lo[1])&(out[:,1]<hi[1]);q=out[sel];pix=xy(q);rgb=np.asarray(im).copy();rgb[pix[:,1],pix[:,0]]=(175,188,204);im=Image.fromarray(rgb);d=ImageDraw.Draw(im);line=xy(path);d.line([tuple(x) for x in line[::3]],fill='#42dfd2',width=3)
for idx,color,text in [(0,'#73fa8b','START'),(-1,'#ff765c','END')]:
 x,y=line[idx];d.ellipse((x-6,y-6,x+6,y+6),fill=color);d.text((x+9,y),text,fill=color)
d.text((30,H+48),'Observed obstacles 0.035-1.4 m; cyan = LIO path',fill='white');d.text((30,H+68),f'Floor alignment: {summary["tilt_deg"]:.1f} deg; grid scale 1 m below',fill='white');d.line((30,H+103,30+scale,H+103),fill='white',width=3)
im.save(p/'field-map.png')
for sec in range(450,593,10):
 i=abs(ts-sec).argmin();print('height',sec,round(path[i,2],3))
