"""Replay scan coverage and rear clearance at each supervisor bumper event."""
import argparse
import json, math
from pathlib import Path
import numpy as np
import rosbag2_py as bag
from tf2_ros import Buffer
from rclpy.time import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from osp_mission.scan_safety import rear_clear
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('run',type=Path)
run=parser.parse_args().run.resolve()
events=list(map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()))
start=next(x['wall_epoch_s'] for x in events if x['event']=='mission_started')
hits=[e['wall_epoch_s'] for e in events if e['event']=='bumper_recovery']
b=bag.SequentialReader();b.open(bag.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('',''))
topics=['/tf_static','/scan','/scan_self_filtered','/hazard_detection','/odom']
types={x.name:get_message(x.type) for x in b.get_all_topics_and_types() if x.name in topics}
b.set_filter(bag.StorageFilter(topics=topics))
buf=Buffer();rows=[];scans={};odom=None;frames=[];i=0
while b.has_next() and i<len(hits):
 topic,raw,ns=b.read_next(); t=ns/1e9; m=deserialize_message(raw,types[topic])
 if topic=='/tf_static':
  for tr in m.transforms:buf.set_transform_static(tr,'bag')
 elif topic.startswith('/scan'):scans[topic]=(t,m)
 elif topic=='/hazard_detection':
  hit=[d.header.frame_id for d in m.detections if d.type==1]
  if hit:frames=hit
 elif topic=='/odom':odom=m
 if t<hits[i] or len(scans)<2 or odom is None:continue
 vals={}
 for k,(stamp,scan) in scans.items():
  tr=buf.lookup_transform('base_link',scan.header.frame_id,Time()).transform
  q=tr.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  r=np.array(scan.ranges);a=scan.angle_min+np.arange(len(r))*scan.angle_increment+yaw
  valid=(r>scan.range_min)&(r<scan.range_max)&np.isfinite(r)
  x=tr.translation.x+r[valid]*np.cos(a[valid]);y=tr.translation.y+r[valid]*np.sin(a[valid])
  angles=np.arctan2(y,x); sign=1 if 'left' in str(frames) else -1 if 'right' in str(frames) else 0
  sector=(abs(angles)<1.55)&(angles*sign>=0)
  vals[k]=dict(min_base_center_m=round(float(np.min(np.hypot(x[sector],y[sector]))),3) if sector.any() else None,
       scan_age_s=round(t-stamp,3),rear_clear=bool(rear_clear(scan,(tr.translation.x,tr.translation.y,yaw))))
 p=odom.pose.pose.position
 rows.append(dict(seconds=round(t-start,2),bump=frames,x=round(p.x,3),y=round(p.y,3),v=round(odom.twist.twist.linear.x,3),scans=vals))
 i+=1
print(json.dumps(rows))
