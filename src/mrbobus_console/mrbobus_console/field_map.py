"""Manual map alignment, bounded scan matching and persistent goal annotations.
No velocity publisher, no navigation action and no TF broadcaster.
"""
import json,math,threading,uuid
from pathlib import Path
import numpy as np

def pose_matrix(x,y,yaw,z=.045):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,z],[0,0,0,1.]])

def transform(points,m):return points@m[:3,:3].T+m[:3,3]

class FieldMap:
    def __init__(self,root):
        self.root=Path(root);self.lock=threading.RLock();self.alignment=None;self.quality=None;self.goals=[];self.tree=None;self.data=None
        path=self.root/'field.npz'
        if not path.exists():return
        a=np.load(path);self.points=a['points'];self.bounds=a['bounds'];self.recorded_path=a['trajectory']
        self.data=json.dumps({'id':'field-20260928','points':np.round(self.points.astype(float),3).tolist(),'bounds':self.bounds.tolist(),'trajectory':np.round(self.recorded_path.astype(float),3).tolist()}).encode()
        goals=self.root/'goals.json'
        if goals.exists():self.goals=json.loads(goals.read_text())
    def validate(self,data):
        if self.data is None:raise ValueError('Карта не загружена')
        vals=[]
        for key in ('x','y','yaw'):
            value=data.get(key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError('Некорректная поза')
            vals.append(float(value))
        x,y,yaw=vals
        if not self.bounds[0,0]<=x<=self.bounds[1,0] or not self.bounds[0,1]<=y<=self.bounds[1,1]:raise ValueError('Точка вне поля')
        return x,y,yaw
    def set_pose(self,data,current):
        x,y,yaw=self.validate(data)
        with self.lock:
            self.alignment=pose_matrix(x,y,yaw)@np.linalg.inv(current)
            self.quality={'method':'ручная поза','verified':False}
        return {'ok':True}
    def goal(self,data):
        x,y,yaw=self.validate(data)
        with self.lock:
            if len(self.goals)>=100:raise ValueError('Не более 100 целей')
            goal={'id':uuid.uuid4().hex[:10],'x':x,'y':y,'yaw':yaw,'label':f'Цель {len(self.goals)+1}'}
            self.goals.append(goal);self.save()
        return {'ok':True,'goal':goal}
    def remove(self,data):
        with self.lock:self.goals=[g for g in self.goals if g['id']!=data.get('id')];self.save()
        return {'ok':True}
    def save(self):
        tmp=self.root/'goals.tmp';tmp.write_text(json.dumps(self.goals));tmp.replace(self.root/'goals.json')
    def status(self,current,fresh):
        with self.lock:
            p=None
            if self.alignment is not None and current is not None and fresh:
                m=self.alignment@current;p={'x':float(m[0,3]),'y':float(m[1,3]),'z':float(m[2,3]),'yaw':math.atan2(m[1,0],m[0,0])}
            return {'loaded':self.data is not None,'pose':p,'goals':list(self.goals),'quality':self.quality,'navigation':False}
    def refine(self,cloud,current):
        from scipy.spatial import cKDTree
        with self.lock:
            if self.alignment is None:raise ValueError('Сначала укажи положение и направление робота')
            initial=self.alignment.copy()
        p=np.array(cloud);p=p[(p[:,2]>.06)&(p[:,2]<1.6)]
        if len(p)<150:raise ValueError('Недостаточно точек для сопоставления')
        p=p[::max(1,len(p)//3000)]
        if self.tree is None:
            target=self.points[(self.points[:,2]>.08)&(self.points[:,2]<1.7)]
            self.target=target;self.tree=cKDTree(target)
        source=transform(p,initial@current);correction=np.eye(4)
        for _ in range(20):
            dist,idx=self.tree.query(source);mask=dist<.3
            if mask.sum()<100:raise ValueError('Облако не совпало: уточни начальную позу')
            threshold=min(.3,float(np.percentile(dist[mask],80)));keep=dist<=threshold
            a,b=source[keep,:2],self.target[idx[keep],:2];ca,cb=a.mean(0),b.mean(0)
            u,_,vh=np.linalg.svd((a-ca).T@(b-cb));rot=vh.T@u.T
            if np.linalg.det(rot)<0:vh[-1]*=-1;rot=vh.T@u.T
            delta=np.eye(4);delta[:2,:2]=rot;delta[:2,3]=cb-rot@ca
            source=transform(source,delta);correction=delta@correction
            if np.linalg.norm(delta[:2,3])<.001 and abs(math.atan2(rot[1,0],rot[0,0]))<.001:break
        dist,_=self.tree.query(source);ratio=float(np.mean(dist<.15));rms=float(np.sqrt(np.mean(dist[dist<.3]**2)))
        before=(initial@current)[:2,3];after=(correction@initial@current)[:2,3];shift=float(np.linalg.norm(after-before));angle=abs(math.atan2(correction[1,0],correction[0,0]))
        if ratio<.45 or rms>.15 or shift>.6 or angle>.4:raise ValueError(f'Совмещение ненадёжно: совпало {ratio:.0%}, RMS {rms:.2f} м. Уточни позу.')
        with self.lock:
            self.alignment=correction@initial;self.quality={'method':'локальное совмещение ICP','verified':True,'overlap':ratio,'rms':rms,'shift':shift}
        return {'ok':True,'quality':self.quality}
