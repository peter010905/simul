"""Evaluator-only separation of map localization drift from marker-relative sensing."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml

from continual_native_curriculum import digest,save


def table(path,columns):
    rows=[[float(row[key]) for key in columns] for row in csv.DictReader(path.open())]
    return np.asarray(rows,dtype=float).reshape((-1,len(columns)))


def rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def monotonic(values):
    # Prefer the last recorded value of duplicate sensor stamps.
    values=values[np.argsort(values[:,0],kind='stable')]
    return values[np.r_[np.diff(values[:,0])>0,True]]


def statistics(values):
    return dict(samples=len(values),minimum=float(np.min(values)),median=float(np.median(values)),
        p90=float(np.quantile(values,.9)),maximum=float(np.max(values))) if len(values) else dict(samples=0)


def analyze(episode):
    evaluation=json.loads((episode/'evaluation.json').read_text())
    scene=Path(evaluation['scenario']);cfg=yaml.safe_load(scene.read_text())
    observations=json.loads((episode/'marker_observations.json').read_text())
    poses=table(episode/'pose_trace.csv',['gt_stamp','gt_x','gt_y','gt_yaw',
        'map_stamp','map_x','map_y','map_yaw','odom_stamp','odom_x','odom_y','odom_yaw'])
    maps=monotonic(poses[:,[4,5,6,7]]);maps[:,3]=np.unwrap(maps[:,3])
    fine=[]
    for path in episode.glob('evaluator_gt_fine*.csv'):
        meta=path.with_suffix('.json')
        if meta.exists() and json.loads(meta.read_text()).get('complete'):
            values=table(path,['sim_seconds','world_x','world_y','world_yaw'])
            if len(values):fine.append((path,values))
    if not fine:raise ValueError('Require a finalized independent fine trajectory segment')
    # Do not extrapolate to observations before this segment was attached.
    truth=monotonic(np.concatenate([values for _,values in fine]));truth[:,3]=np.unwrap(truth[:,3])
    tx,ty,theta=evaluation['ground_truth_audit']['world_to_map'];R=rotation(theta)
    truth[:,1:3]=truth[:,1:3]@R.T+[tx,ty];truth[:,3]+=theta
    targets={int(value['id']):np.array([value['x'],value['y']])@R.T+[tx,ty] for value in cfg['targets']}
    by_id={}
    for ident,target in targets.items():
        raw=[];corrected=[];position_error=[];heading_error=[];stamps=[]
        for value in observations:
            stamp=value['stamp']
            if value['id']!=ident or not max(truth[0,0],maps[0,0])<=stamp<=min(truth[-1,0],maps[-1,0]):continue
            mapped=np.array([np.interp(stamp,maps[:,0],maps[:,key]) for key in [1,2,3]])
            actual=np.array([np.interp(stamp,truth[:,0],truth[:,key]) for key in [1,2,3]])
            point=np.array(value['map_point'])[:2]
            # Counterfactual evaluator transform only. Never supplied to the detector or planner.
            relative=point-mapped[:2]
            projected=actual[:2]+rotation(actual[2]-mapped[2])@relative
            raw.append(np.linalg.norm(point-target));corrected.append(np.linalg.norm(projected-target))
            position_error.append(np.linalg.norm(mapped[:2]-actual[:2]))
            heading_error.append(abs(math.degrees((mapped[2]-actual[2]+math.pi)%(2*math.pi)-math.pi)))
            stamps.append(stamp)
        by_id[str(ident)]=dict(recorded_map_marker_error_m=statistics(raw),
            counterfactual_pose_corrected_marker_error_m=statistics(corrected),
            robot_map_position_error_m=statistics(position_error),robot_map_heading_error_deg=statistics(heading_error),
            observed_stamp_range_sim_s=[min(stamps),max(stamps)] if stamps else None)
    paths=[scene,episode/'evaluation.json',episode/'marker_observations.json',episode/'pose_trace.csv',*[p for p,_ in fine]]
    return dict(evidence_type='finalized_native_sensor_replay_with_evaluator_only_counterfactual_pose',
        episode=str(episode),native_success=evaluation['success'],native_seconds=evaluation['elapsed_seconds'],
        native_coordinate_errors_m=evaluation['ground_truth_audit']['position_errors_m'],by_id=by_id,
        fine_trajectory_segment_sim_s=[float(truth[0,0]),float(truth[-1,0])],
        policy_or_result_modified=False,ground_truth_supplied_to_policy=False,
        limits=['Pose correction is a diagnostic counterfactual,not a deployed fix or native success',
            'Map poses are interpolated from coarse audit samples;rapid motion or SLAM jumps add interpolation uncertainty',
            'Observations outside finalized fine trajectory coverage are omitted,not extrapolated',
            'Small counterfactual error supports localization drift as dominant;it does not prove a particular SLAM setting will fix it'],
        source_sha256={str(path):digest(path) for path in paths})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();save(args.output,analyze(args.episode.resolve()))
