"""Compare choices on recorded candidate sets and check side-surface opportunities."""
import argparse
import bisect
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.time import Time
from tf2_ros import Buffer
from osp_mission import edge_policy
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.wall_route import WallLook, camera_pose, penalize_routes, yaw


def main(run):
    events=list(map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()))
    start=next(e['wall_epoch_s'] for e in events if e['event']=='mission_started')
    reader=bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('',''))
    topics=['/odom','/tf','/tf_static','/scan','/mission/processed_camera','/mission/status']
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in topics}
    reader.set_filter(bag.StorageFilter(topics=topics))
    headings=[]; poses=[]; tf=[]; scans=[]; info=None; action=''; static=Buffer()
    while reader.has_next():
        topic,data,ns=reader.read_next()
        t=ns/1e9-start
        if t>120:break
        m=deserialize_message(data,types[topic])
        if topic=='/tf_static':
            for tr in m.transforms:static.set_transform_static(tr,'bag')
        elif topic=='/tf':
            for tr in m.transforms:
                if (tr.header.frame_id.strip('/'),tr.child_frame_id.strip('/'))==('map','odom'):
                    p=tr.transform.translation;a=-yaw(tr.transform.rotation)
                    tf.append((t,(-math.cos(a)*p.x+math.sin(a)*p.y,-math.sin(a)*p.x-math.cos(a)*p.y,a)))
        elif topic=='/mission/processed_camera':info=m
        elif topic=='/mission/status':action=json.loads(m.data).get('active_action','')
        elif t>=0 and topic=='/odom':
            p=m.pose.pose.position;poses.append((t,p.x,p.y));headings.append(yaw(m.pose.pose.orientation))
        elif t>=0 and topic=='/scan' and action=='nav' and (not scans or t-scans[-1][0]>=1.):
            scans.append((t,m))
    # Three distinct visits must include an excursion at least 0.9 m from the cell center.
    visits=Counter();last={}
    for t,x,y in poses[::5]:
        cell=(math.floor(x/.6),math.floor(y/.6))
        if cell not in last or any(math.hypot(px-(cell[0]+.5)*.6,py-(cell[1]+.5)*.6)>=.9
                                   for pt,px,py in poses if last[cell]<pt<t):
            visits[cell]+=1
        last[cell]=t
    mem=ExplorationMemory();pos=0;times=[v[0] for v in tf]; decisions=[]
    for file in sorted((run/'artifacts/policy_decisions').glob('*.npz')):
        with np.load(file) as d:
            t=float(d['elapsed'])
            if t>120:break
            while pos<len(poses) and poses[pos][0]<=t:
                pt,x,y=poses[pos];mem.observe(x,y,pt);pos+=1
            i=bisect.bisect_right(times,t)-1
            if not pos or int(d['selected_index'])<0:continue
            if i>=0:
                transform=tf[i][1]
            else:
                pt,x,y=poses[pos-1]
                if t-pt>.25:continue
                a=headings[pos-1]-float(d['heading']);mx,my=d['robot']
                transform=(x-math.cos(a)*mx+math.sin(a)*my,y-math.sin(a)*mx-math.cos(a)*my,a)
            data=d['data'];res=float(d['resolution']);origin=d['origin']
            _,prev=edge_policy.options(data,res,origin,d['robot'],float(d['heading']),d['visits'],d['viewed'],
                                      return_paths=True,point_sampler=lambda *args:[],start_ignore_radius=.30)
            gs=[]
            for row in d['candidates']:
                x,y=row[:2];gs.append(dict(cell=(int((y-origin[1])/res),int((x-origin[0])/res)),
                                          features=row[4:].copy()))
            before=edge_policy.choose(gs,d['weights'])
            if before is not gs[int(d['selected_index'])]:continue
            penalize_routes(gs,prev,data.shape,res,origin,transform,mem.cells)
            after=edge_policy.choose(gs,d['weights'])
            decisions.append((round(t,1),after is not before))
    wall=WallLook();hits=[]
    if info is not None and scans:
        cam=camera_pose(static.lookup_transform('base_link',info.header.frame_id,Time()).transform)
        laser=static.lookup_transform('base_link',scans[0][1].header.frame_id,Time()).transform
        for t,m in scans:
            p=wall.select(m,(laser.translation.x,laser.translation.y,yaw(laser.rotation)),cam,
                          2*math.atan(info.width/(2*info.k[0])),t)
            if p is not None:hits.append(round(t,1))
    print(json.dumps(dict(seconds=120,three_visit_cells=sum(n>=3 for n in visits.values()),
        max_cell_visits=max(visits.values()),route_choices=decisions,nav_scan_samples=len(scans),
        surface_samples=len(hits),first_surface_s=hits[:5]),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    main(parser.parse_args().run)
