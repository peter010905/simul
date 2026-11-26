"""Post-run metrics. Scene, truth and recordings are opened only by this evaluator."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml

def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line] if path.exists() else []

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def navigation_switches(log):
    """Compare successive Nav2 translation destinations actually sent by the mission."""
    pattern=r'\[mission\]: (?:Navigation goal \(([-.\d]+), ([-.\d]+)\)|Cancel nav: ([\w]+))'
    previous=None
    cancel=None
    sends=switches=preempted=0
    for x,y,reason in re.findall(pattern,log):
        if reason:
            cancel=reason
            continue
        current=(float(x),float(y))
        if previous is not None and math.dist(previous,current)>.15:
            switches+=1
            preempted+=int(cancel=='marker_hint')
        previous=current
        cancel=None
        sends+=1
    return sends,switches,preempted

def evaluate(folder):
    scene=json.loads((folder/'scene.json').read_text())
    start=yaml.safe_load((folder/'scene.yaml').read_text())['start']
    metrics=json.loads((folder/'metrics.json').read_text())
    targets={int(t['id']):[t['x'],t['y']] for t in scene['targets']}
    native=folder/'native'
    events=lines(folder/'events.jsonl')
    ack=next(row['sim'] for row in events if row['event']=='start_ack')
    with (folder/'headings.csv').open() as stream:
        poses=np.array([[float(row[k]) for k in ('elapsed_sim_s','x','y','yaw_rad')] for row in csv.DictReader(stream)])
    poses=poses[np.argsort(poses[:,0],kind='stable')]
    stamps,ix=np.unique(poses[:,0],return_index=True)
    poses=poses[ix]
    end=min(metrics['elapsed_sim_s'],stamps[-1])
    times=np.unique(np.r_[max(0,stamps[0]),np.arange(max(.2,math.ceil(stamps[0]*5)/5),end,.2),end])
    trajectory=np.stack([np.interp(times,stamps,poses[:,i]) for i in (1,2)],axis=1)
    distance=np.linalg.norm(np.diff(trajectory,axis=0),axis=1).sum()
    goals=lines(native/'hybrid_goals.jsonl')
    extensions=lines(native/'marker_nbv.jsonl')
    decisions=[]
    for f in sorted((native/'policy_decisions').glob('*.npz')):
        with np.load(f) as d:
            if int(d['selected_index'])>=0:
                decisions.append((float(d['elapsed']),d['robot'].copy(),float(d['heading']),
                                  d['candidates'][int(d['selected_index']),:2].copy()))
    attempts=[]
    rels=[g['t']-ack for g in goals]
    anon_looks=0
    for i,g in enumerate(goals):
        if g['kind']!='print':
            continue
        t=rels[i]
        # Match the actual selected goal (including repeated positions) to its decision.
        valid=[d for d in decisions if math.dist(d[3],(g['x'],g['y']))<.03 and abs(d[0]-t)<2.]
        if not valid:
            raise ValueError(f'No aligned decision for {folder.name} goal {i}')
        d=min(valid,key=lambda item:abs(item[0]-t))
        ext=next((row for row in extensions if abs(row['elapsed']-d[0])<.1 and 'nbv_rgb' in row),{})
        if ext.get('stationary',math.dist(d[1],d[3])<.01):
            anon_looks+=1
            continue
        limit=(rels[i+1] if i+1<len(rels) else end)+3.
        found=[int(ident) for ident,when in metrics['registered_s'].items() if t<=when<=limit]
        attempt=dict(t=t,registered_in_window=found)
        if ext.get('point_map') is not None:
            j=int(np.argmin(abs(stamps-t)))
            if abs(stamps[j]-t)>.25:
                raise ValueError('Missing synchronized truth')
            a=poses[j,3]-d[2]
            c,s=math.cos(a),math.sin(a)
            guess=poses[j,1:3]+np.array([[c,-s],[s,c]])@(np.asarray(ext['point_map'])-d[1])
            attempt['nearest_true_marker_m']=min(math.dist(guess,q) for q in targets.values())
        attempts.append(attempt)
    log=(folder/'launch.log').read_text(errors='replace')
    cancels=Counter(re.findall(r'\[mission\]: Cancel nav: ([\w]+)',log))
    nav_sends,goal_switches,preempted=navigation_switches(log)
    reached={int(x) for x in metrics['registered_and_within_s']}
    ids=set(metrics['final_ids'])
    h=digest(folder/'world/osp_studio.sdf')
    assert h==metrics['world_sha256']
    return dict(run=folder.name,world_sha256=h,scene_sha256=digest(folder/'scene.yaml'),start=start,
                policy_revision=(folder/'policy_revision.txt').read_text().strip(),ids=sorted(ids),within_and_read=sorted(reached),
                success=ids==set(targets) and reached==set(targets),reason=metrics['reason'],
                completion_s=metrics['pair_time_s'],observed_s=metrics['elapsed_sim_s'],path_m=round(float(distance),3),
                min_distances=metrics['min_ground_truth_m'],bumper=metrics['bumper_events'],
                dispatched=len(goals),nav_sends=nav_sends,goal_switches=goal_switches,perception_preemptions_to_different_goal=preempted,midroute_switch_requests=cancels.get('marker_hint',0),nav_cancels=dict(cancels),
                candidate_approaches=len(attempts),unproductive_approaches=sum(not a['registered_in_window'] for a in attempts),
                off_target_gt=(sum(a['nearest_true_marker_m']>.5 for a in attempts)
                               if all('nearest_true_marker_m' in a for a in attempts) else None),
                approach_audit=attempts,candidate_stationary_looks=anon_looks,
                multiview_goals=sum(len(a.get('nbv_gain',{}))>=2 for a in extensions),
                quality_relooks=sum(len(a.get('nbv_gain',{}))==1 for a in extensions),
                rgb_at_last_goal=extensions[-1].get('rgb_evidence') if extensions else None,
                exposures_at_last_goal=extensions[-1].get('exposures') if extensions else None,
                unexpected_fallbacks=log.count('Hybrid proposal unavailable'),
                simulation_clock_start=ack,registered_at=metrics['registered_s'])

def compare(base,version):
    b,v=map(evaluate,(base,version))
    for k in ('world_sha256','scene_sha256','start'):
        if b[k]!=v[k]:
            raise ValueError('Paired input mismatch: '+k)
    return dict(baseline=b,candidate=v)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('base',type=Path)
    p.add_argument('version',type=Path,nargs='?')
    a=p.parse_args()
    print(json.dumps(compare(a.base,a.version) if a.version else evaluate(a.base),indent=2,allow_nan=False))
