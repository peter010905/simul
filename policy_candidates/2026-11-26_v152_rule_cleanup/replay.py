"""Stateless matched selector replay on physical SLAM and costmap snapshots.

No ROS node, publisher, action client or driving. This is not closed-loop
navigation. Marker/box tracks and historical exclusions are not reconstructed.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
import rosbag2_py as bag
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.r32_drive import grid_from_message


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=16)
    a=p.parse_args()
    start=next(r['wall_epoch_s'] for r in map(json.loads,
        (a.run/'artifacts/startup.jsonl').read_text().splitlines()) if r['event']=='mission_started')
    snapshots=sorted((a.run/'artifacts/policy_decisions').glob('*.npz'))
    snapshots=[snapshots[i] for i in np.unique(np.linspace(0,len(snapshots)-1,min(a.samples,len(snapshots))).astype(int))]
    reader=bag.SequentialReader()
    reader.open(bag.StorageOptions(uri=str(a.run/'rosbag'),storage_id='mcap'),bag.ConverterOptions('cdr','cdr'))
    wanted={'/tf','/odom','/global_costmap/costmap','/local_costmap/costmap','/mission/processed_camera'}
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in wanted}
    reader.set_filter(bag.StorageFilter(topics=list(types)))
    def stream():
        while reader.has_next():
            n,raw,t=reader.read_next()
            yield n,deserialize_message(raw,types[n]),t/1e9-start
    it=iter(stream());item=next(it,None)
    costs={};odom=None;transform=None;camera=None;rows=[]
    for path in snapshots:
        snap=np.load(path);elapsed=float(snap['elapsed'])
        while item is not None and item[2]<=elapsed:
            name,m,t=item
            if name=='/tf':
                for tr in m.transforms:
                    if (tr.header.frame_id,tr.child_frame_id)==('map','odom'):
                        q=tr.transform.rotation
                        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                        c,s=math.cos(yaw),math.sin(yaw)
                        x,y=tr.transform.translation.x,tr.transform.translation.y
                        transform=(-c*x-s*y,s*x-c*y,-yaw)
            elif name=='/odom':
                q=m.pose.pose.orientation
                odom=(t,m.pose.pose.position.x,m.pose.pose.position.y,
                      math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            elif name=='/mission/processed_camera':
                if m.k[0]>0:camera=(float(m.k[0]),2*math.atan(m.width/(2*m.k[0])))
            else:
                costs[name]=(m.header.frame_id,grid_from_message(m),t)
            item=next(it,None)
        mt=transform;source='recorded_tf'
        if mt is None and odom is not None and abs(odom[0]-elapsed)<=.20:
            yaw=odom[3]-float(snap['heading']);c,s=math.cos(yaw),math.sin(yaw)
            x,y=snap['robot'];mt=(odom[1]-c*x+s*y,odom[2]-s*x-c*y,yaw)
            source='paired_map_pose_and_odometry'
        global_cost=costs.get('/global_costmap/costmap')
        if mt is None or global_cost is None or elapsed-global_cost[2]>3.:
            rows.append(dict(snapshot=path.name,elapsed=elapsed,skipped='missing_fresh_geometry'))
            continue
        g=Grid(snap['data'],float(snap['resolution']),(*snap['origin'],0.))
        s=CenterWall();s.survey.camera=camera
        local=costs.get('/local_costmap/costmap')
        if local and -.1<=elapsed-local[2]<=1.5:s.local_box_cost=local[:2]
        choices=[dict(x=x,y=y,yaw=yaw,scan_angle=scan,distance=distance,
                      features=np.array([visual,frontier,distance,turn,revisit]))
                 for x,y,yaw,scan,visual,frontier,distance,turn,revisit in snap['candidates']]
        before=time.perf_counter()
        goal=s.decide(g,global_cost[1],(*snap['robot'],float(snap['heading'])),mt,
                      snap['viewed'],elapsed,600.,lambda _:False,choices)
        ms=1000*(time.perf_counter()-before)
        safe,_=g.reachable(snap['robot'],.32)
        legal=None
        if goal and not goal.get('stationary'):
            cell=g.index(goal['x'],goal['y'])
            legal=bool(g.inside(*cell) and safe[cell])
        rows.append(dict(snapshot=path.name,elapsed=round(elapsed,3),transform_source=source,
            selector_ms=round(ms,3),reason=s.reason,kind=goal.get('r32_kind') if goal else None,
            goal_xy=[goal['x'],goal['y']] if goal else None,normal_endpoint_safe=legal))
    times=[r['selector_ms'] for r in rows if 'selector_ms' in r]
    a.output.write_text(json.dumps(dict(run=a.run.name,physical_drive=False,
        evidence='stateless_recorded_map_and_costmap_selector_replay',
        limitations=['veto/history and tracked objects not reconstructed','no Nav2 action feasibility proof',
                     'not a marker-detection or closed-loop success measurement'],
        selected=sum(r.get('kind') is not None for r in rows),samples=len(times),
        unsafe_endpoints=sum(r.get('normal_endpoint_safe') is False for r in rows),
        reasons=dict(Counter(r.get('reason',r.get('skipped')) for r in rows)),
        median_ms=float(np.median(times)) if times else None,rows=rows),indent=2)+'\n')


if __name__=='__main__':main()
