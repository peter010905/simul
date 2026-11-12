"""Evaluator-only physical proximity at recorded approach-credit timestamps.

The policy never imports this module. A historical closest pass elsewhere in
the episode cannot validate a later, incorrectly localized approach credit.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def physical_at_stamp(trajectory,target,stamp,speed_bound=.6):
    rows=np.asarray(trajectory,dtype=float);target=np.asarray(target,dtype=float)
    if (rows.ndim!=2 or rows.shape[1]!=3 or len(rows)<2 or not np.isfinite(rows).all()
            or not np.all(np.diff(rows[:,0])>0)):
        return dict(verified=False,reason='invalid_or_missing_independent_trajectory')
    if not np.isfinite(stamp) or not rows[0,0]<=stamp<=rows[-1,0]:
        return dict(verified=False,reason='credit_timestamp_outside_recorded_trajectory')
    right=int(np.searchsorted(rows[:,0],stamp,side='left'))
    if right<len(rows) and abs(rows[right,0]-stamp)<1e-9:
        distance=float(np.linalg.norm(rows[right,1:]-target))
        return dict(verified=distance<=1.5,stamp_sim_s=float(stamp),
            actual_distance_m=distance,conservative_distance_upper_m=distance,method='exact_recorded_GT_sample')
    left=right-1;dt=rows[right,0]-rows[left,0]
    if not 0<dt<=.30:
        return dict(verified=False,reason='independent_pose_gap_too_large',gap_s=float(dt))
    observed_speed=float(np.linalg.norm(rows[right,1:]-rows[left,1:])/dt)
    if observed_speed>speed_bound:
        return dict(verified=False,reason='motion_exceeds_registered_speed_bound',observed_segment_speed_m_s=observed_speed)
    fraction=(stamp-rows[left,0])/dt
    position=rows[left,1:]*(1-fraction)+rows[right,1:]*fraction
    distance=float(np.linalg.norm(position-target))
    upper=float(min(np.linalg.norm(rows[left,1:]-target)+speed_bound*(stamp-rows[left,0]),
                    np.linalg.norm(rows[right,1:]-target)+speed_bound*(rows[right,0]-stamp)))
    return dict(verified=upper<=1.5,stamp_sim_s=float(stamp),actual_distance_m=distance,
        conservative_distance_upper_m=upper,bracketing_stamps_sim_s=[float(rows[left,0]),float(rows[right,0])],
        registered_speed_bound_m_s=speed_bound,method='interpolated_GT_with_speed_margin',
        reason=None if upper<=1.5 else 'credited_pose_outside_radius_or_insufficient_sampling_margin')


def audit(episode,scenario):
    evaluation=episode/'evaluation.json';truth=episode/'ground_truth_trajectory.csv'
    events=episode/'marker_events.jsonl'
    result=json.loads(evaluation.read_text());cfg=yaml.safe_load(scenario.read_text())
    trajectory=[[float(r[k]) for k in ('sim_seconds','world_x','world_y')]
        for r in csv.DictReader(truth.open())]
    fine_files=sorted(episode.glob('evaluator_gt_fine_*.csv'))
    fine=[]
    for file in fine_files:
        fine.extend([float(r[k]) for k in ('sim_seconds','world_x','world_y')]
            for r in csv.DictReader(file.open()))
    fine=sorted({row[0]:row for row in fine}.values())
    def at(point,stamp):
        use_fine=len(fine)>=2 and fine[0][0]<=stamp<=fine[-1][0]
        report=physical_at_stamp(fine if use_fine else trajectory,point,stamp)
        report['trajectory_source']='fine_evaluator_trace' if use_fine else 'original_0_2s_evaluator_trace'
        return report
    journal=[json.loads(line) for line in events.read_text().splitlines() if line.strip()]
    confirmed={int(e['id']) for e in journal if e.get('event')=='id_confirmed'}
    estimated={int(e['id']) for e in journal if e.get('event')=='approach_estimated_within_1_5m'}
    approach=result.get('approach',{});rows=[]
    for target in cfg['targets']:
        ident=int(target['id']);point=[target['x'],target['y']]
        credit=approach.get('approach_evidence',{}).get(str(ident))
        pose=at(point,credit['stamp']) if credit else dict(verified=False,reason='no_map_pose_credit')
        witness=approach.get('range_witness_evidence',{}).get(str(ident),{})
        samples=witness.get('observations',[])
        physical=[at(point,s['stamp']) for s in samples]
        stamps=[s['stamp'] for s in samples]
        witness_passed=bool(len(samples)==3 and all(b-a>=.15-1e-8 for a,b in zip(stamps,stamps[1:]))
            and all(s.get('distance_m',99)<=1.4 for s in samples)
            and all(s['verified'] for s in physical))
        rows.append(dict(id=ident,id_confirmed_in_journal=ident in confirmed,
            estimated_approach_in_journal=ident in estimated,map_pose_credit=pose,
            range_witness_physical_samples=physical,range_witness_verified=witness_passed,
            physical_credit_verified=bool(ident in confirmed and (pose['verified'] or witness_passed))))
    sources=[scenario,evaluation,truth,events,*fine_files]
    return dict(evidence_type='independent_GT_at_saved_policy_credit_timestamps',
        episode=str(episode),required_actual_radius_m=1.5,ground_truth_available_to_policy=False,
        registered_speed_bound_m_s=.6,
        sampling_limit='Between samples the upper distance bound assumes speed<=0.6m/s; segment speeds above it or gaps>0.30s are unverified',
        historical_closest_pass_alone_is_sufficient=False,
        all_three_physical_credits_verified=len(rows)==3 and all(r['physical_credit_verified'] for r in rows),
        all_three_recognition_and_approach_events_logged=len(rows)==3 and all(
            r['id_confirmed_in_journal'] and r['estimated_approach_in_journal'] for r in rows),
        rows=rows,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','scenario','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();report=audit(args.episode.resolve(),args.scenario.resolve())
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256')},indent=2))
