"""Read a recorded mission in receipt order. The experiment has no ROS action clients."""
import argparse
from collections import Counter, defaultdict, deque
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py as bag
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer

from osp_mission.wall_route import camera_pose, yaw, wrap
from surface_memory import SurfaceMemory, local_observations


def project(base, offset):
    c,s=math.cos(base[2]),math.sin(base[2])
    x,y,a=offset
    return (base[0]+c*x-s*y,base[1]+s*x+c*y,wrap(base[2]+a))


def header_time(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9


def nearest(rows,t,tolerance):
    row=min(rows,key=lambda item:abs(item[0]-t),default=None)
    return row[1] if row and abs(row[0]-t)<=tolerance else None


def replay(run,output):
    events=[json.loads(line) for line in (run/'artifacts/startup.jsonl').read_text().splitlines()]
    start=next(e['wall_epoch_s'] for e in events if e['event']=='mission_started')
    stop=min(e['wall_epoch_s'] for e in events if e['event'] in ('stopped','safety_stop','mission_finished'))
    override_file=run/'artifacts/exploration_memory.jsonl'
    overrides=[e['elapsed_seconds'] for e in (json.loads(x) for x in override_file.read_text().splitlines())
               if e.get('marker_override')] if override_file.exists() else []
    snapshots=[]
    for path in sorted((run/'artifacts/policy_decisions').glob('*.npz')):
        with np.load(path) as snap:
            snapshots.append((start+float(snap['elapsed']),path))
    reader=bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('',''))
    rgb='/oakd/rgb/preview/image_raw'; info='/oakd/rgb/preview/camera_info';scan='/scan_self_filtered'
    topics=[rgb,info,scan,'/odom','/tf_static','/mission/marker_observations','/mission/status']
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    reader.set_filter(bag.StorageFilter(topics=topics))
    static=Buffer();bridge=CvBridge();memory=SurfaceMemory();cal=None
    poses=deque(maxlen=60);scans=deque(maxlen=15);path=[];decisions=[];audits={}
    seen_baseline=set();coverage_checks=set();marker_positions=defaultdict(list);approached=set()
    counts=Counter();last=-1e20;frame_cache={};pos=0
    def cached(frame, optical=False):
        key=frame,optical
        if key not in frame_cache:
            tr=static.lookup_transform('base_link',frame,Time()).transform
            frame_cache[key]=camera_pose(tr) if optical else (tr.translation.x,tr.translation.y,yaw(tr.rotation))
        return frame_cache[key]
    def decide(t,filename):
        base=nearest(poses,t,.25)
        if base is None or cal is None:
            counts['missing_decision_pose']+=1;return
        elapsed=t-start
        with np.load(filename) as snap:
            index=int(snap['selected_index'])
            if index<0 or any(abs(elapsed-k)<.2 for k in overrides):return
            counts['baseline_decisions']+=1
            # At each decision compare with the actual saved directional coverage array.
            rot=base[2]-float(snap['heading']);c,s=math.cos(rot),math.sin(rot)
            rotation=np.array([[c,-s],[s,c]])
            res=float(snap['resolution']);grid=snap['data'];view=snap['viewed']
            map_robot=snap['robot'];origin=snap['origin'];mapped=[]
            for k in memory.pending(t):
                f=memory.faces[k]
                cell=(((f.xy-base[:2])@rotation+map_robot-origin)/res).astype(int)
                x,y=cell
                if not (2<=x<grid.shape[1]-2 and 2<=y<grid.shape[0]-2):continue
                normal=f.normal@rotation
                side=np.argmax([-normal[1],normal[0],normal[1],-normal[0]])
                dx,dy=((0,-1),(1,0),(0,1),(-1,0))[side]
                wall=grid[y-1:y+2,x-1:x+2]>50
                free=grid[y-1+dy:y+2+dy,x-1+dx:x+2+dx]==0
                if not (wall&free).any():continue
                mapped.append(k)
                coverage_checks.add(k)
                if (view[side,y-1:y+2,x-1:x+2]&wall&free).any():seen_baseline.add(k)
            decision=memory.choose(snap,base,t,cal.k[0],mapped)
            if decision:
                decision['mission_s']=round(elapsed,2);decisions.append(decision)
    while reader.has_next():
        topic,payload,ns=reader.read_next()
        t=ns/1e9
        if t>stop:break
        while pos<len(snapshots) and snapshots[pos][0]<=t:
            decide(*snapshots[pos]);pos+=1
        msg=deserialize_message(payload,types[topic])
        if topic=='/tf_static':
            for tr in msg.transforms:static.set_transform_static(tr,'bag')
        elif topic==info:cal=msg
        elif topic==scan:scans.append((header_time(msg),msg))
        elif topic=='/odom':
            p=msg.pose.pose
            poses.append((header_time(msg),(p.position.x,p.position.y,yaw(p.orientation))))
        elif t>=start and topic=='/mission/status':
            approached.update(json.loads(msg.data).get('approached_ids',[]))
        elif t>=start and topic=='/mission/marker_observations':
            e=json.loads(msg.data)
            if e.get('odom_point') is None:continue
            ident=e['id'];point=np.array(e['odom_point'][:2])
            marker_positions[ident].append(point)
            if ident in audits:continue
            base=nearest(poses,e['stamp'],.25)
            if base is None:continue
            close=[]
            for idx,f in enumerate(memory.faces):
                dist=np.linalg.norm(f.xy-point)
                if dist<=.30 and f.normal@(np.array(base[:2])-f.xy)>0 and f.queued is not None:
                    close.append((dist,idx))
            active=[(dist,idx) for dist,idx in close if not memory.faces[idx].confirmed and t-memory.faces[idx].last_poor<=120]
            target_ids={d['face_id'] for d in decisions}
            audits[ident]=dict(active_prior_faces=len(active), previously_targeted=any(idx in target_ids for _,idx in close), time_s=round(t-start,2),prior_weak_face_count=len(close),
                               prior_s=round(t-memory.faces[min(close)[1]].queued,2) if close else None)
        elif t>=start and topic==rgb:
            stamp=header_time(msg)
            if stamp-last<.25:continue
            last=stamp;counts['sampled']+=1
            laser=nearest(scans,stamp,.18);base=nearest(poses,stamp,.15)
            if laser is None or base is None or cal is None or cal.width!=msg.width or cal.height!=msg.height:continue
            laser_base=nearest(poses,header_time(laser),.15)
            if laser_base is None:continue
            try:
                cam=project(base,cached(cal.header.frame_id,True))
                origin=project(laser_base,cached(laser.header.frame_id))
            except Exception:
                counts['missing_camera_transform']+=1;continue
            path.append(base[:2]);image=bridge.imgmsg_to_cv2(msg,'mono8')
            memory.update(local_observations(laser,origin,cam,cal,image),cam,t)
            counts['used']+=1
    faces=memory.faces;poor=[f for f in faces if f.queued is not None]
    rows=[dict(x=float(f.xy[0]),y=float(f.xy[1]),nx=float(f.normal[0]),ny=float(f.normal[1]),
               queued_s=None if f.queued is None else round(f.queued-start,2),confirmed=f.confirmed,
               revisited=f.revisited,reasons=dict(f.reasons)) for f in faces]
    marker_xy={str(k):np.median(v,axis=0).tolist() for k,v in marker_positions.items()}
    result=dict(run=run.name,mission_s=round(stop-start,1),counts=dict(counts),faces=len(faces),queued=len(poor),
                later_good=sum(f.confirmed for f in poor),different_view=sum(f.revisited for f in poor),
                unresolved=sum(not f.confirmed for f in poor),matched_saved_coverage=len(coverage_checks),
                marked_viewed_by_baseline=len(seen_baseline),reranked=len(decisions),proposal_types=dict(Counter(d["kind"] for d in decisions)),marker_audit=audits,
                actual_approached=sorted(approached),reason_face_counts=dict(Counter(reason for f in poor for reason in f.reasons)))
    (output/(run.name+'.json')).write_text(json.dumps(dict(summary=result,proposals=decisions,faces=rows,
                                                           odom_path=path,marker_xy=marker_xy),separators=(',',':')))
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('runs',type=Path,nargs='+')
    args=parser.parse_args();args.output.mkdir(exist_ok=True,parents=True);cv2.setNumThreads(1)
    for run in args.runs:replay(run,args.output)
