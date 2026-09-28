#!/usr/bin/env python3
"""Offline ROS bag export: timestamped camera video and LIO-registered cloud.
Run with ROS Jazzy sourced. Does not publish topics or access motor hardware.
"""
import argparse,json,subprocess
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage,PointCloud2
from tf2_msgs.msg import TFMessage

def records(path):
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    while r.has_next():yield r.read_next()

def matrix(p,q):
    x,y,z,w=q.x,q.y,q.z,q.w
    m=np.eye(4);m[:3,:3]=[[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],[2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],[2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]]
    m[:3,3]=[p.x,p.y,p.z];return m

def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

def main():
    ap=argparse.ArgumentParser();ap.add_argument('bag',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    frames=a.output/'frames';frames.mkdir(exist_ok=True);poses=[];wheels=[];times=[];counts={};tf={};video=[]
    for topic,data,t in records(a.bag):
        counts[topic]=counts.get(topic,0)+1
        if topic in ('/lio/odometry','/odom'):
            msg=deserialize_message(data,Odometry);m=matrix(msg.pose.pose.position,msg.pose.pose.orientation)
            if topic=='/lio/odometry':times.append(stamp(msg));poses.append(m)
            else:wheels.append([stamp(msg),*m[:3,3],np.arctan2(m[1,0],m[0,0])])
        elif topic=='/tf_static':
            for x in deserialize_message(data,TFMessage).transforms:tf[(x.header.frame_id,x.child_frame_id)]=matrix(x.transform.translation,x.transform.rotation)
        elif topic=='/camera/image/compressed':
            msg=deserialize_message(data,CompressedImage);name=f'{len(video):06d}.jpg';(frames/name).write_bytes(bytes(msg.data));video.append((name,stamp(msg)))
    print('Pass 1:',counts,flush=True)
    if video:
        listing=[]
        for i,(name,t) in enumerate(video):
            dt=video[i+1][1]-t if i+1<len(video) else .2
            listing.extend([f"file 'frames/{name}'",f'duration {max(.001,dt):.9f}'])
        listing.append(f"file 'frames/{video[-1][0]}'")
        (a.output/'video.concat').write_text('\n'.join(listing)+'\n')
        subprocess.run(['ffmpeg','-nostdin','-y','-loglevel','error','-f','concat','-safe','0','-i',str(a.output/'video.concat'),'-fps_mode','vfr','-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output/'camera.mp4')],check=True)
    if not poses:raise RuntimeError('No LIO poses')
    times=np.array(times);poses=np.array(poses);order=np.argsort(times);times=times[order];poses=poses[order]
    li=tf[('unilidar_lidar','unilidar_imu')];bl=tf[('base_link','unilidar_lidar')]
    base=poses@np.linalg.inv(bl@li);xyz=base[:,:3,3];yaw=np.unwrap(np.arctan2(base[:,1,0],base[:,0,0]))
    np.savez_compressed(a.output/'trajectory.npz',time=times,base=base,wheel=np.array(wheels))
    chunks=[];map_points=np.empty((0,3));used=0;skipped=0;gaps=[]
    for topic,data,t in records(a.bag):
        if topic!='/unilidar/cloud':continue
        msg=deserialize_message(data,PointCloud2);ts=stamp(msg);i=min(np.searchsorted(times,ts),len(times)-1)
        if i and abs(times[i-1]-ts)<abs(times[i]-ts):i-=1
        gap=abs(times[i]-ts);gaps.append(gap)
        if gap>.15:skipped+=1;continue
        fields={f.name:f.offset for f in msg.fields};endian='>' if msg.is_bigendian else '<'
        dt=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[k] for k in ('x','y','z')],'itemsize':msg.point_step})
        raw=np.ndarray((msg.height,msg.width),dtype=dt,buffer=msg.data,strides=(msg.row_step,msg.point_step)).reshape(-1)
        p=np.stack([raw[k] for k in ('x','y','z')],axis=1)[::2];p=p[np.isfinite(p).all(axis=1)];r=np.linalg.norm(p,axis=1);p=p[(r>.5)&(r<10)]
        m=poses[i]@np.linalg.inv(li);p=p@m[:3,:3].T+m[:3,3];chunks.append(p);used+=1
        if len(chunks)>=60:
            p=np.concatenate([map_points,*chunks]);_,idx=np.unique(np.floor(p/.05).astype(np.int32),axis=0,return_index=True);map_points=p[idx];chunks=[]
    if chunks:
        p=np.concatenate([map_points,*chunks]);_,idx=np.unique(np.floor(p/.05).astype(np.int32),axis=0,return_index=True);map_points=p[idx]
    np.savez_compressed(a.output/'map.npz',points=map_points)
    with (a.output/'map.ply').open('wb') as f:
        f.write(f'ply\nformat binary_little_endian 1.0\nelement vertex {len(map_points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n'.encode());f.write(map_points.astype('<f4').tobytes())
    sampled=np.r_[0,np.flatnonzero(np.diff(np.floor(times)))+1]
    summary={'bag':str(a.bag),'duration_s':float(times[-1]-times[0]),'topics':counts,'camera_frames':len(video),'camera_duration_s':video[-1][1]-video[0][1] if video else 0,'lio_bounds_m':[xyz.min(0).tolist(),xyz.max(0).tolist()],'endpoint_displacement_m':float(np.linalg.norm(xyz[-1]-xyz[0])),'path_at_1hz_m':float(np.linalg.norm(np.diff(xyz[sampled],axis=0),axis=1).sum()),'max_position_step_m':float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).max()),'max_yaw_step_deg':float(np.rad2deg(abs(np.diff(yaw))).max()),'max_pose_gap_s':float(np.diff(times).max()),'clouds_used':used,'clouds_skipped':skipped,'cloud_pose_gap_p99_s':float(np.percentile(gaps,99)),'map_points':len(map_points),'registration':'nearest recorded LIO pose; no per-point deskew, no loop closure','video_start_unix':video[0][1] if video else None}
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2),flush=True)
if __name__=='__main__':main()
