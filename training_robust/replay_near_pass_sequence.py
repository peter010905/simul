"""Inspect look budgets along finalized training paths,without changing a path.

This shadow retains the original observed maps and coverage,uses estimated look
duration,and does not synthesize any camera or recognition evidence. It is not
a native counterfactual or an estimate of mission time saved.
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

from continual_native_curriculum import digest,save


def table(path,keys):
    values=np.asarray([[float(row[key]) for key in keys] for row in csv.DictReader(path.open())])
    values=values[np.isfinite(values).all(axis=1)]
    values=values[np.argsort(values[:,0],kind='stable')]
    return values[np.r_[np.diff(values[:,0])>0,True]]


def calibration(episode,mapped):
    offsets=[];rotations=[]
    for row in json.loads((episode/'marker_observations.json').read_text()):
        stamp=row['stamp']
        if not mapped[0,0]<=stamp<=mapped[-1,0]:continue
        body=np.array([np.interp(stamp,mapped[:,0],mapped[:,k]) for k in (1,2,3)])
        origin=row.get('camera_origin_map');point=row.get('camera_point')
        if origin is None or point is None:continue
        delta=np.asarray(origin[:2])-body[:2];c,s=math.cos(body[2]),math.sin(body[2])
        offsets.append([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
        delta=np.asarray(row['map_point'][:2])-np.asarray(origin[:2])
        yaw=math.atan2(delta[1],delta[0])-math.atan2(-point[0],point[2])-body[2]
        rotations.append((yaw+math.pi)%(2*math.pi)-math.pi)
    if not offsets:raise ValueError('No accepted sensor observations for camera extrinsics')
    return dict(body_offset_xy_m=np.median(offsets,axis=0).tolist(),
        yaw_offset_rad=float(np.median(rotations)),samples=len(offsets),
        input='Accepted sensor map/camera observations and recorded TF;no scenario or target truth')


def load_snapshots(episode):
    journal=[json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
    snapshots=[]
    for path in sorted((episode/'policy_decisions').glob('*.npz')):
        with np.load(path) as data:
            elapsed=float(data['elapsed']);record=min(journal,key=lambda row:abs(row['elapsed_seconds']-elapsed))
            if abs(record['elapsed_seconds']-elapsed)>.25:continue
            snapshots.append(dict(path=path,elapsed=elapsed,protected=record['marker_override'],
                focal=record.get('inspection_focal_px') or 0.,data=data['data'].copy(),
                viewed=data['viewed'].copy(),origin=data['origin'].copy(),resolution=float(data['resolution'])))
    if not snapshots:raise ValueError('Require matched observed-map and decision journals')
    return snapshots


def replay(job,module,camera_info,step,offset):
    episode=Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    evaluation=json.loads((episode/'evaluation.json').read_text())
    mapped=table(episode/'pose_trace.csv',['map_stamp','map_x','map_y','map_yaw'])
    odometry=table(episode/'pose_trace.csv',['odom_stamp','odom_x','odom_y','odom_yaw'])
    mapped[:,3]=np.unwrap(mapped[:,3]);odometry[:,3]=np.unwrap(odometry[:,3])
    camera=calibration(episode,mapped);snapshots=load_snapshots(episode)
    recording=evaluation['recording']
    # Video clock and status are asynchronously sampled;probe offsets bracket
    # half a second of start-time uncertainty instead of asserting exact timing.
    inferred_start=recording['last_sim_seconds']-recording['last_mission_status']['elapsed_seconds']+offset
    if any(s['focal'] and not math.isclose(s['focal'],camera_info['k'][0],rel_tol=1e-6) for s in snapshots):
        raise ValueError('Shared camera calibration differs from recorded per-decision focal length')
    fov=camera_info['horizontal_fov_rad'];policy=module.NearPassInspection()
    current=None;cache_checks=0;events=[];geometry_available=0;eligible=0;busy_until=-math.inf
    original_faces,original_distance=module.boundary_faces,module.distance_transform_edt
    def optical(pose):
        c,s=math.cos(pose[2]),math.sin(pose[2]);x,y=camera['body_offset_xy_m']
        return (float(pose[0]+c*x-s*y),float(pose[1]+s*x+c*y),float(pose[2]+camera['yaw_offset_rad']))
    try:
        earliest=max(mapped[0,0],odometry[0,0],inferred_start+15.)
        latest=min(mapped[-1,0],odometry[-1,0],inferred_start+evaluation['elapsed_seconds'])
        for stamp in np.arange(earliest,latest,step):
            elapsed=float(stamp-inferred_start)
            candidates=[s for s in snapshots if s['elapsed']<=elapsed]
            if not candidates:continue
            snapshot=candidates[-1]
            if snapshot['protected'] or elapsed-snapshot['elapsed']<3. or stamp<busy_until:continue
            eligible+=1
            body=np.array([np.interp(stamp,mapped[:,0],mapped[:,k]) for k in (1,2,3)])
            odom=np.array([np.interp(stamp,odometry[:,0],odometry[:,k]) for k in (1,2,3)])
            arguments=(snapshot['data'],snapshot['resolution'],snapshot['origin'],optical(body),body[:2],
                snapshot['viewed'],snapshot['focal'],fov,optical(odom),float(stamp),elapsed,600.-elapsed)
            # Pure deterministic memoization reduces interference with native
            # runs. Verify the first candidate on every raster against originals.
            changed=current is not snapshot
            if changed:
                module.boundary_faces, module.distance_transform_edt=original_faces,original_distance
                expected=policy.choose(*arguments)
                faces=original_faces(snapshot['data'],snapshot['resolution'])
                mask=snapshot['data']==0;clearance=original_distance(mask)
                def cached_faces(data,resolution):
                    if data is not snapshot['data'] or resolution!=snapshot['resolution']:
                        raise ValueError('Memoized face input changed')
                    return faces
                def cached_distance(value):
                    if not np.array_equal(value,mask):raise ValueError('Memoized clearance input changed')
                    return clearance
                module.boundary_faces,module.distance_transform_edt=cached_faces,cached_distance
                current=snapshot
            choice=policy.choose(*arguments)
            if changed:
                if choice!=expected:raise ValueError('Memoization changed policy selection')
                cache_checks+=1
            if choice is None:
                if module.NearPassInspection().choose(*arguments) is not None:geometry_available+=1
                continue
            geometry_available+=1
            policy.reserve(choice,float(stamp));busy_until=float(stamp)+choice['estimated_sim_s']
            # Estimated phase duration only;no camera/coverage evidence added.
            policy.finish(busy_until,'shadow_estimated_duration_no_new_coverage')
            events.append(dict(stamp_sim_s=float(stamp),elapsed_sim_s=elapsed,snapshot=str(snapshot['path']),
                camera_map_pose=list(optical(body)),choice=choice))
    finally:module.boundary_faces,module.distance_transform_edt=original_faces,original_distance
    return dict(shape=job['scenario']['arena']['shape'],seed=job['scenario']['seed'],episode=str(episode),
        original_native_outcome=job['outcome'],start_offset_sim_s=offset,inferred_mission_start_sim_s=inferred_start,
        calibration=camera,eligible_probes=eligible,geometry_opportunities_without_history_or_budget=geometry_available,
        requested_looks=len(events),estimated_phase_time_sim_s=policy.spent,history=events,
        memoization_equivalence_checks=cache_checks,
        original_final_recognition_ids=evaluation['ground_truth_audit']['found_ids'],
        input_hashes={str(p):digest(p) for p in [episode/'evaluation.json',episode/'pose_trace.csv',
            episode/'marker_observations.json',episode/'exploration_memory.jsonl',*[s['path'] for s in snapshots]]})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('campaign','workspace','output','camera-info'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--step',type=float,default=2.)
    args=parser.parse_args();workspace=args.workspace.resolve();sys.path.insert(0,str(workspace/'src/osp_mission'))
    module_path=workspace/'src/osp_mission/osp_mission/near_pass_inspection.py'
    spec=importlib.util.spec_from_file_location('near_pass_sequence_shadow',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    state=json.loads((args.campaign/'state.json').read_text());camera_info=json.loads(args.camera_info.read_text())
    cases=[]
    for job in state['jobs']:
        if job['status']!='done' or job['phase']!='train':continue
        for offset in (-.5,0.,.5):
            case=replay(job,module,camera_info,args.step,offset);cases.append(case)
            print(json.dumps({k:case[k] for k in ['shape','seed','start_offset_sim_s','requested_looks','estimated_phase_time_sim_s']}),flush=True)
    save(args.output,dict(evidence_type='recorded_training_path_look_budget_shadows_not_new_native_missions',
        step_sim_s=args.step,cases=cases,qualification_or_holdout_consumed=False,ground_truth_available_to_policy=False,
        new_recognition_or_coverage_generated=False,new_completion_or_time_saving_verified=False,
        limits=['Original path,rasters and camera coverage retained;requested looks do not create new views or change motion',
            'Look duration is an estimate;later Nav2 reorientation and actual interruption latency are not simulated',
            'Between planner snapshots exact active action is unavailable;ordinary exploration legs include potential post-arrival scans and dwell',
            'Video clock/status infer mission start with0.5sbracketing;frame poses are interpolated from coarse audit samples',
            'Current camera extrinsics inferred independently per episode;shared earlier verified960x720FOV checked against all recorded focal lengths'],
        source_sha256={str(Path(__file__).resolve()):digest(Path(__file__)),str(module_path):digest(module_path),
            str(args.camera_info.resolve()):digest(args.camera_info)}))
