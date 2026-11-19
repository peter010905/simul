"""Summarize finalized matched native Gazebo runs, scoring only box-mounted IDs."""
import argparse
import json
from collections import Counter
import csv
import math
from pathlib import Path


def load(path):
    return json.loads(path.read_text())


def face_dwell(folder, targets):
    # Rover heading is a proxy for camera aim. Occlusion and actual decodes
    # are assessed separately; simulator ground truth is evaluator-only.
    dwell = {k:0. for k in targets}
    previous = None
    with (folder/'headings.csv').open() as file:
        for row in csv.DictReader(file):
            t,x,y,yaw = map(float,(row['elapsed_sim_s'],row['x'],row['y'],row['yaw_rad']))
            dt = min(.10,max(0.,t-previous)) if previous is not None else 0.
            previous = t
            for k,marker in targets.items():
                dx,dy = x-marker['x'],y-marker['y']
                distance = math.hypot(dx,dy)
                front = -dx*math.cos(marker['yaw'])-dy*math.sin(marker['yaw'])
                direction = math.atan2(-dy,-dx)
                if 0 < distance <= 1.5 and front>.4*distance and math.cos(yaw-direction)>.866:
                    dwell[k] += dt
    return {k:round(v,2) for k,v in dwell.items()}


def metrics(folder, targets):
    audit = load(folder/'audit.json')
    trial = load(folder/'metrics.json')
    reached = {k:v for k,v in audit['registered_and_within_s'].items() if k in targets}
    registered = {k:v for k,v in audit['registered_s'].items() if k in targets}
    goals = [json.loads(line) for line in (folder/'native/hybrid_goals.jsonl').read_text().splitlines()]
    end = audit['simulation_start_clock'] + audit['elapsed_sim_s']
    box_goal_intervals = sum(max(0.,min(following,end)-max(goal['t'],audit['simulation_start_clock']))
                             for goal,following in zip(goals,[g['t'] for g in goals[1:]]+[end])
                             if goal['kind'] in ('box_face_view','box_look'))
    return {
        'box_goal_to_next_dispatch_s': round(box_goal_intervals,3),
        'approx_printed_face_dwell_s': face_dwell(folder,targets),
        'run': folder.name, 'revision': (folder/'policy_revision.txt').read_text().strip(),
        'registered_s': registered, 'registered_within_1_5m_s': reached,
        'both_boxes_s': max(reached.values()) if len(reached)==len(targets) else None,
        'elapsed_sim_s': audit['elapsed_sim_s'], 'elapsed_wall_s': audit['elapsed_wall_s'],
        'stop_reason': audit['reason'], 'bumper_events': audit['bumper_events'],
        'goal_counts': dict(Counter(audit['hybrid_goal_types'])),
        'camera': trial['last_policy_status'].get('camera_diagnostics',{}),
        'camera_decoded_frames': {k:audit['offline_rgb'][k]['frames'] for k in sorted(targets)},
        'world_sha256': audit['world_sha256']
    }


def compare(baseline, candidate):
    scene = load(baseline/'scene.json')
    if scene != load(candidate/'scene.json'):
        raise ValueError('Scene mismatch')
    boxes = {b['name'] for b in scene['obstacles']}
    targets = {str(t['id']):t for t in scene['targets'] if t['attached_to'] in boxes}
    hosts = {k:t['attached_to'] for k,t in targets.items()}
    if len(targets)!=2 or len(set(hosts.values()))!=2:
        raise ValueError('Expected two distinct marked boxes')
    old, new = (metrics(folder,targets) for folder in (baseline,candidate))
    if old['world_sha256']!=new['world_sha256'] or old['revision']==new['revision']:
        raise ValueError('Expected different policies in identical worlds')
    delta = None
    if old['both_boxes_s'] is not None and new['both_boxes_s'] is not None:
        delta = new['both_boxes_s']-old['both_boxes_s']
    return dict(seed=scene['seed'], box_markers=hosts, baseline=old, candidate=new,
                both_boxes_delta_s=delta, runtime_delta_sim_s=new['elapsed_sim_s']-old['elapsed_sim_s'])


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('baseline',type=Path)
    p.add_argument('candidate',type=Path)
    args=p.parse_args()
    print(json.dumps(compare(args.baseline,args.candidate),indent=2))
