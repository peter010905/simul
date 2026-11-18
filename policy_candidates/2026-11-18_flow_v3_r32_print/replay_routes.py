"""Paired ordinary-goal comparison on archived decision grids and latest received costmap."""
from collections import Counter
from pathlib import Path
from types import SimpleNamespace as NS
import argparse
import math
from bisect import bisect_right
import json

import numpy as np
from nav_msgs.msg import OccupancyGrid
from osp_mission.navigation_flow import NavigationFlow
from osp_mission.hybrid import Hybrid,yaw
from replay import stream


def routes(run):
    start=next(e['wall_epoch_s'] for e in map(json.loads,(run/'artifacts/startup.jsonl').read_text().splitlines()) if e['event']=='mission_started')
    decisions=sorted((run/'artifacts/policy_decisions').glob('*.npz'))
    memory_file=run/'artifacts/exploration_memory.jsonl'
    protected=[e['elapsed_seconds'] for e in map(json.loads,memory_file.read_text().splitlines()) if e.get('marker_override')] if memory_file.exists() else []
    tf_samples=[]
    for _,msg,received in stream(run,['/tf']):
        for tr in msg.transforms:
            if (tr.header.frame_id,tr.child_frame_id)==('map','odom'):
                a=-yaw(tr.transform.rotation);c,s=math.cos(a),math.sin(a)
                x,y=tr.transform.translation.x,tr.transform.translation.y
                tf_samples.append((received,(-c*x+s*y,-s*x-c*y,a)))
    tf_times=[t for t,_ in tf_samples]
    costmaps=iter(stream(run,['/global_costmap/costmap']))
    row=next(costmaps,None);cost=None;counts=Counter(); examples=[]
    hybrid=Hybrid()
    for path in decisions:
        with np.load(path) as data:
            sec=float(data['elapsed']); t=start+sec
            while row and row[2]<=t:
                cost=row[1];row=next(costmaps,None)
            if any(abs(sec-e)<.5 for e in protected):
                counts['reserved_marker_decision']+=1;continue
            m=OccupancyGrid();g=data['data'].astype(np.int8);m.data=g.flatten().tolist()
            m.info.height,m.info.width=g.shape;m.info.resolution=float(data['resolution'])
            m.info.origin.position.x,m.info.origin.position.y=data['origin'].tolist()
            m.info.origin.orientation.w=1.
            clock=NS(now=lambda:NS(nanoseconds=int(1e9*t)))
            owner=NS(map=m,get_clock=lambda:clock,now=lambda:t,started=start,targets={},excluded=data['visits'].tolist(),
                     p=lambda k: {'map_frame':'map','duration':600.}[k],
                     exploration_memory=NS(views=[]),navigation_flow=NavigationFlow())
            hybrid.drive.costmap=cost
            idx=int(data['selected_index']);base=None
            if idx>=0:
                x,y,a=data['candidates'][idx,:3]
                base=dict(x=float(x),y=float(y),yaw=float(a))
            pose=(*data['robot'],float(data['heading']))
            # Snapshot coordinates stay in the recorded map; never fabricate a cross-frame pose.
            i=bisect_right(tf_times,t)-1
            transform=tf_samples[i][1] if i>=0 and t-tf_times[i]<=1.5 else None
            selected=hybrid.select(owner,base,g,data['viewed'],pose,transform)
            kind='no_goal' if selected is None else selected.get('r32_kind','flow')
            counts[kind]+=1
            if kind not in ('flow','no_goal') and len(examples)<3:
                examples.append(dict(t=round(sec,2),kind=kind,xy=[selected['x'],selected['y']]))
            if selected:
                hybrid.drive.dispatched(selected)
                if 'box_odom' in selected:
                    hybrid.last_box=t
                    hybrid.seen_boxes.append((selected['box_odom'],t))
    return dict(run=run.name,counts=counts,examples=examples)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    a=p.parse_args()
    print(json.dumps([routes(a.root/r) for r in ('20261118T025452-9e81af1edd','20261118T083739-28a113496c','20261118T082353-9f8e401d05')],indent=2))
