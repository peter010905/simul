"""Estimate horizontal angle scale from background tracks during in-place turns."""
from pathlib import Path
import argparse
import json
import math

import cv2
import numpy as np
import rosbag2_py as bag
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

RGB='/oakd/rgb/preview/image_raw'
INFO='/oakd/rgb/preview/camera_info'


def stream(run, topics):
    reader=bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('',''))
    types={r.name:get_message(r.type) for r in reader.get_all_topics_and_types() if r.name in topics}
    reader.set_filter(bag.StorageFilter(topics=topics))
    while reader.has_next():
        topic,raw,_=reader.read_next()
        msg=deserialize_message(raw,types[topic])
        yield topic,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,msg


def background_pairs(run, seconds=90):
    rows=[]
    for _,t,m in stream(run,['/odom']):
        p=m.pose.pose
        q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rows.append((t,p.position.x,p.position.y,yaw))
    rows=np.array(sorted(rows))
    rows[:,3]=np.unwrap(rows[:,3])
    bridge=CvBridge()
    first=previous=intrinsic=None
    pairs=[]
    for topic,t,msg in stream(run,[RGB,INFO]):
        if topic==INFO:
            intrinsic=(msg.k[0],msg.k[2])
            continue
        if intrinsic is None or not rows[0,0]<t<rows[-1,0]:
            continue
        if first is None:first=t
        if t-first>seconds:break
        if previous and t-previous[0]<.39:continue
        pose=np.array([np.interp(t,rows[:,0],rows[:,i]) for i in (1,2,3)])
        image=bridge.imgmsg_to_cv2(msg,'mono8')
        old=previous
        previous=t,image,pose
        if not old:continue
        t0,im0,p0=old
        turn=pose[2]-p0[2]
        if not (.055<abs(turn)<.36 and t-t0<.58 and np.linalg.norm(pose[:2]-p0[:2])<.025):continue
        pts=cv2.goodFeaturesToTrack(im0,140,.015,7)
        if pts is None or len(pts)<12:continue
        options=dict(winSize=(21,21),maxLevel=3)
        nxt,ok,_=cv2.calcOpticalFlowPyrLK(im0,image,pts,None,**options)
        back,bok,_=cv2.calcOpticalFlowPyrLK(image,im0,nxt,None,**options)
        valid=(ok.ravel()!=0)&(bok.ravel()!=0)&(np.linalg.norm((back-pts)[:,0,:],axis=1)<.8)
        a,b=pts[valid,0,:],nxt[valid,0,:]
        if len(a)<12:continue
        # Reject independently moving foreground using a common background homography.
        _,inliers=cv2.findHomography(a,b,cv2.RANSAC,2.)
        if inliers is None:continue
        a,b=a[inliers.ravel()!=0],b[inliers.ravel()!=0]
        if len(a)>=12 and np.ptp(a[:,0])>90:
            pairs.append((t-first,turn,a[:,0],b[:,0]))
    return intrinsic,pairs


def measure(run):
    (fx,cx),pairs=background_pairs(run)
    scales=np.arange(.75,2.501,.01)
    per_pair=[]
    for _,turn,u,v in pairs:
        f=fx*scales[:,None]
        errors=np.abs(np.arctan((v-cx)/f)-np.arctan((u-cx)/f)-turn)
        per_pair.append(np.median(errors,axis=1))
    costs=np.array(per_pair)
    med=np.median(costs,axis=0)
    best=int(np.argmin(med))
    per_scale=scales[np.argmin(costs,axis=1)]
    print(run.name,len(pairs),'scale',scales[best],np.percentile(per_scale,[25,50,75]).tolist(),flush=True)
    return dict(run=run.name,pairs=len(pairs),features=sum(len(p[2]) for p in pairs),
                fx=fx,cx=cx,scale=round(float(scales[best]),3),
                scale_iqr=[round(v,3) for v in np.percentile(per_scale,[25,75])],
                scale_grid=scales.round(3).tolist(),median_error_deg=np.rad2deg(med).round(4).tolist())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path);p.add_argument('runs',type=Path,nargs='+')
    args=p.parse_args();cv2.setNumThreads(1)
    args.output.write_text(json.dumps([measure(r) for r in args.runs],indent=2)+'\n')
