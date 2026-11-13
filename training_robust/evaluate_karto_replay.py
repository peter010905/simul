"""Evaluator-only trajectory errors for saved SLAM component replays."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def evaluate(episode,directory):
    evaluation=json.loads((episode/'evaluation.json').read_text())
    tx,ty,angle=evaluation['ground_truth_audit']['world_to_map']
    c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
    with (episode/'pose_trace.csv').open() as stream:raw=list(csv.DictReader(stream))
    gt=sorted({float(r['gt_stamp']):(float(r['gt_x']),float(r['gt_y'])) for r in raw}.items())
    times=np.array([r[0] for r in gt]);xy=np.array([r[1] for r in gt])
    arms=[]
    for path in sorted(directory.glob('*.online.csv')):
        with path.open() as stream:rows=list(csv.DictReader(stream))
        values=[]
        for row in rows:
            stamp=float(row['stamp_sim_s'])
            if not times[0]<=stamp<=times[-1]:continue
            truth=rotation@np.array([np.interp(stamp,times,xy[:,0]),np.interp(stamp,times,xy[:,1])])+[tx,ty]
            values.append((stamp,float(np.linalg.norm(truth-[float(row['map_x']),float(row['map_y'])]))))
        if not values:raise ValueError('No synchronized evaluator positions')
        errors=np.array([v[1] for v in values]);name=path.name.removesuffix('.online.csv')
        events=directory/(name+'.events.log');text=events.read_text() if events.exists() else ''
        arms.append(dict(name=name,retained_scan_count=len(rows),accepted_scan_count=sum(int(r['accepted']) for r in rows),
            matched_evaluator_samples=len(values),median_error_m=float(np.median(errors)),p95_error_m=float(np.percentile(errors,95)),
            maximum_error_m=float(max(errors)),last_stamp_sim_s=values[-1][0],last_error_m=values[-1][1],
            first_error_over_0_5m=next((dict(stamp=stamp,error_m=error) for stamp,error in values if error>.5),None),
            loop_closures=text.count(' BEGIN '),loop_checks_near_original_jump=[line for line in text.splitlines() if line.startswith('124.')],
            trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    return dict(evidence_type='SLAM_component_replay_evaluated_against_separate_truth',episode=str(episode),arms=arms,
        native_mission=False,policy_ground_truth_inputs=False,original_results_modified=False,
        limits=['Recorded accepted scans are a subset of the original ROS scan stream.',
                'Translation errors use interpolated evaluator body positions and the original world-to-map alignment.',
                'No new navigation, recognition, approach, contact or completion result is produced.'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','directory','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();report=evaluate(args.episode.resolve(),args.directory.resolve())
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps([{k:v for k,v in row.items() if k in ('name','maximum_error_m','last_error_m','loop_closures')} for row in report['arms']]))
