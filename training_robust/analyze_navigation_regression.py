"""Evaluator-only comparison of finalized same-scene navigation ablations."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import re

import numpy as np

from analyze_revisit import analyze as revisit_analysis


def motion_intervals(episode):
    rows=list(csv.DictReader((episode/'pose_trace.csv').open()))
    total=still=turning=longest=run=0.
    for before,after in zip(rows,rows[1:]):
        dt=float(after['gt_stamp'])-float(before['gt_stamp'])
        if not 0<dt<=1.:run=0.;continue
        distance=math.hypot(float(after['gt_x'])-float(before['gt_x']),
                            float(after['gt_y'])-float(before['gt_y']))
        turn=abs((float(after['gt_yaw'])-float(before['gt_yaw'])+math.pi)%(2*math.pi)-math.pi)
        total+=dt
        if distance/dt<.03:
            if turn/dt<.05:
                still+=dt;run+=dt;longest=max(longest,run)
            else:turning+=dt;run=0.
        else:run=0.
    return dict(observed_sim_s=total,stationary_sim_s=still,
        stationary_turn_sim_s=turning,longest_stationary_interval_s=longest,
        caveat='Stationary camera verification and observation turns can be necessary; not automatically wasted time')


def decision_health(episode):
    total=empty=occupied=0
    for path in sorted((episode/'policy_decisions').glob('*.npz')):
        with np.load(path) as data:
            total+=1;empty+=len(data['candidates'])==0
            cell=np.floor((data['robot']-data['origin'])/float(data['resolution'])).astype(int)
            if 0<=cell[1]<data['data'].shape[0] and 0<=cell[0]<data['data'].shape[1]:
                occupied+=int(data['data'][cell[1],cell[0]]>50)
    return dict(total=total,no_candidate_decisions=int(empty),robot_in_occupied_map_cell=int(occupied))


def summarize(job):
    output=Path(job['output']);episode=output/'episodes'/str(job['scenario']['seed'])
    result=json.loads((episode/'evaluation.json').read_text())
    log=(episode/'launch.log').read_text(errors='replace')
    attempts=Counter(map(int,re.findall(r'Approaching confirmed ID (\d+)',log)))
    probes={path.stem:json.loads(path.read_text()) for path in output.glob('deadlock_probe*.json')}
    return dict(workspace=job['workspace'],outcome=job['outcome'],
        independent_audit=json.loads((output/'episode_audit.json').read_text()),
        floor_sensor_gate=job.get('floor_sensor_gate'),
        runtime_configuration_gate=job.get('runtime_configuration_gate'),
        decision_health=decision_health(episode),motion=motion_intervals(episode),
        revisits=revisit_analysis(episode),approach_requests_by_id=dict(attempts),
        repeated_approach_request_count=sum(max(0,v-1) for v in attempts.values()),
        repeated_approach_caveat='Repeated attempts can be necessary after failed approaches; count alone is not proof of wasted visits',
        latched_physical_range_ids=sorted(map(int,result.get('approach',{}).get('range_witness_evidence',{}))),
        collision_ahead_recoveries=log.count('Collision Ahead'),
        local_costmap_observations=probes)


def compare(parent,regression):
    prior=json.loads((parent/'state.json').read_text());state=json.loads((regression/'state.json').read_text())
    if state['status']!='completed_navigation_regression' or any(j['status']!='done' for j in state['jobs']):
        raise ValueError('Require finalized native recordings and independent audits')
    baseline=prior['jobs'][0]
    if any(j['scenario']['sha256']!=baseline['scenario']['sha256'] or
           j['parameters']!=baseline['parameters'] for j in state['jobs']):
        raise ValueError('Comparison must preserve physical scene and policy weights')
    rows=[summarize(j) for j in [baseline,*state['jobs']]]
    return dict(evidence_type='new_native_same_scene_ablation',training_samples=False,
        generalization_verified=False,production_promoted=False,rows=rows,
        limits=['One native run per arm; no population success-rate estimate',
            'Only local costmap A versus A plus physical-approach witness B isolates the witness change',
            'Simulation scheduling can change trajectories despite identical scenario and weights',
            'Ground truth used only for this evaluator-side analysis, never by the policy'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','regression','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();result=compare(args.parent.resolve(),args.regression.resolve())
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
