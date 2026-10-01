"""Bounded local plane fit for low obstacles on approximately planar indoor floors."""
import numpy as np
from scipy.spatial import cKDTree


def separate_ground(points,floor_points=None):
    p=np.asarray(points);p=p[np.isfinite(p).all(axis=1)]
    r=np.linalg.norm(p[:,:2],axis=1)
    fp=p if floor_points is None else np.asarray(floor_points)
    fr=np.linalg.norm(fp[:,:2],axis=1)
    q=fp[np.isfinite(fp).all(axis=1)&(fr>.35)&(fr<2.5)&(fp[:,2]>-.35)&(fp[:,2]<.02)]
    if len(q):
        _,unique=np.unique(np.floor(q/.025).astype(np.int32),axis=0,return_index=True);q=q[unique]
    if len(q)<100:raise ValueError('Недостаточно точек пола')
    rng=np.random.default_rng(42)
    if len(q)>1800:q=q[rng.choice(len(q),1800,replace=False)]
    best=np.zeros(len(q),bool)
    for _ in range(80):
        a=q[rng.choice(len(q),3,replace=False)];n=np.cross(a[1]-a[0],a[2]-a[0]);norm=np.linalg.norm(n)
        if norm<1e-8 or abs(n[2])/norm<.97:continue
        keep=abs((q-a[0])@(n/norm))<.012
        if keep.sum()>best.sum():best=keep
    f=q[best]
    if len(f)<100 or len(f)<.2*len(q) or min(np.std(f[:,:2],axis=0))<.10:raise ValueError('Плоскость пола не подтверждена')
    coef=np.linalg.lstsq(np.c_[f[:,:2],np.ones(len(f))],f[:,2],rcond=None)[0]
    if np.linalg.norm(coef[:2])>.25 or not -.3<coef[2]<.01:raise ValueError('Недопустимый наклон/высота пола')
    height=p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@coef
    # Exclude chassis returns; do not infer free rays from absent returns.
    outside=(abs(p[:,0])>.19)|(abs(p[:,1])>.21)
    # Empirical side echoes: present in 56/58 scans over 0.5m and 96deg
    # motion in bag-002, and verified with physically clear robot sides.
    # This narrow mask is specific to the current L2 mounting, not a range cutoff.
    self_echo=(p[:,0]>.064)&(p[:,0]<.164)&(abs(p[:,1])>.21)&(abs(p[:,1])<.36)&(p[:,2]>-.08)&(p[:,2]<.14)
    outside &= ~self_echo
    obstacles=p[(height>.035)&(height<1.4)&outside&(r>.22)]
    # Sparse near-floor outliers must not become single-cell lethal obstacles.
    # At least three returns within 8cm; this does not certify thin-object detection.
    if len(obstacles):
        support=cKDTree(obstacles).query_ball_point(obstacles,.08,return_length=True)
        oh=obstacles[:,2]-np.c_[obstacles[:,:2],np.ones(len(obstacles))]@coef
        obstacles=obstacles[(oh>=.15)|(support>=3)]
    return obstacles,{'plane':coef.tolist(),'inliers':len(f),'candidates':len(q),'obstacles':len(obstacles)}


def floor_clearing_ranges(points,plane,obstacles,extra_floor=None):
    """Clear only to observed floor; cap rays at currently detected obstacles."""
    p=np.asarray(points);p=p[np.isfinite(p).all(axis=1)]
    h=p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@np.asarray(plane)
    q=p[abs(h)<.015]
    if extra_floor is not None:q=np.concatenate([q,np.asarray(extra_floor)])
    result=np.zeros(360)
    def polar(a):
        return (np.floor((np.arctan2(a[:,1],a[:,0])+np.pi)/(2*np.pi)*360).astype(int)%360,np.linalg.norm(a[:,:2],axis=1))
    idx,d=polar(q);valid=(d>.22)&(d<3.5);np.maximum.at(result,idx[valid],d[valid])
    nearest=np.full(360,np.inf);idx,d=polar(obstacles);np.minimum.at(nearest,idx,d)
    result=np.minimum(result,nearest-.025);result[result<.22]=np.inf
    return result.astype(np.float32)


def traversable_lawn_mask(points, plane, map_from_base, bounds, max_step=.06):
    """Only observed low steps in the explicitly traversable lawn rectangle."""
    p=np.asarray(points)
    world=p@map_from_base[:3,:3].T+map_from_base[:3,3]
    h=p[:,2]-np.c_[p[:,:2],np.ones(len(p))]@np.asarray(plane)
    return (np.isfinite(p).all(axis=1)&(h>=-.015)&(h<=max_step)&
            (world[:,0]>=bounds[0])&(world[:,0]<=bounds[1])&
            (world[:,1]>=bounds[2])&(world[:,1]<=bounds[3]))
