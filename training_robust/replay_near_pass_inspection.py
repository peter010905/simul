"""Replay nearby-face look opportunities using a prior recorded SLAM raster.

Selection uses recorded map/odom poses,observed occupancy and viewed-face state.
Only the evaluator annotation afterwards opens target truth. No new path,
image,recognition,approach credit or completion result is simulated.
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

from continual_native_curriculum import digest,save


def poses(path,keys):
    rows=np.array([[float(row[key]) for key in keys] for row in csv.DictReader(path.open())])
    rows=rows[np.argsort(rows[:,0],kind='stable')]
    rows=rows[np.r_[np.diff(rows[:,0])>0,True]];rows[:,3]=np.unwrap(rows[:,3])
    return rows


def replay(episode,snapshot,workspace,camera_diagnosis,start,stop):
    sys.path.insert(0,str(workspace/'src/osp_mission'))
    module_path=workspace/'src/osp_mission/osp_mission/near_pass_inspection.py'
    spec=importlib.util.spec_from_file_location('near_pass_shadow',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    trace=episode/'pose_trace.csv';mapped=poses(trace,['map_stamp','map_x','map_y','map_yaw'])
    odometry=poses(trace,['odom_stamp','odom_x','odom_y','odom_yaw'])
    journal=[json.loads(line) for line in (episode/'exploration_memory.jsonl').read_text().splitlines()]
    camera=json.loads(camera_diagnosis.read_text())
    if camera['episode']!=str(episode):raise ValueError('Require camera extrinsics inferred from this same episode')
    # The extrinsic offset is inferred from accepted images,not target truth.
    offset=camera['inferred_camera_extrinsic_xy_m'];yaw_offset=camera['inferred_camera_yaw_offset_rad']
    results=[]
    with np.load(snapshot) as data:
        elapsed=float(data['elapsed']);record=min(journal,key=lambda row:abs(row['elapsed_seconds']-elapsed))
        if abs(record['elapsed_seconds']-elapsed)>.25 or record['marker_override']:
            raise ValueError('Require a matched ordinary exploration navigation snapshot')
        focal=record['inspection_focal_px'];fov=2*math.radians(camera['actual_horizontal_half_fov_deg'])
        if not max(mapped[0,0],odometry[0,0])<=start<stop<=min(mapped[-1,0],odometry[-1,0]):
            raise ValueError('Do not extrapolate recorded TF/odom poses')
        for stamp in np.arange(start,stop+.001,1.):
            body=np.array([np.interp(stamp,mapped[:,0],mapped[:,i]) for i in (1,2,3)])
            odom=np.array([np.interp(stamp,odometry[:,0],odometry[:,i]) for i in (1,2,3)])
            def optical(pose):
                c,s=math.cos(pose[2]),math.sin(pose[2])
                return (float(pose[0]+c*offset[0]-s*offset[1]),
                    float(pose[1]+s*offset[0]+c*offset[1]),float(pose[2]+yaw_offset))
            observed_camera=optical(body)
            policy=module.NearPassInspection()
            choice=policy.choose(data['data'],float(data['resolution']),data['origin'],observed_camera,body[:2],
                data['viewed'],focal,fov,optical(odom),float(stamp),elapsed,600.-elapsed)
            results.append(dict(stamp_sim_s=float(stamp),camera_map_pose=list(observed_camera),choice=choice))
    # Evaluator-only annotation after the policy has returned all choices.
    evaluation=json.loads((episode/'evaluation.json').read_text());scene=Path(evaluation['scenario'])
    cfg=yaml.safe_load(scene.read_text());tx,ty,angle=evaluation['ground_truth_audit']['world_to_map']
    c,s=math.cos(angle),math.sin(angle)
    for row in results:
        if row['choice'] is None:continue
        camera_pose=row['camera_map_pose'];annotations=[]
        for target in cfg['targets']:
            x=tx+c*target['x']-s*target['y'];y=ty+s*target['x']+c*target['y']
            before=(math.atan2(y-camera_pose[1],x-camera_pose[0])-camera_pose[2]+math.pi)%(2*math.pi)-math.pi
            after=(before-row['choice']['turn_rad']+math.pi)%(2*math.pi)-math.pi
            annotations.append(dict(id=target['id'],map_camera_distance_m=math.hypot(x-camera_pose[0],y-camera_pose[1]),
                bearing_before_deg=math.degrees(before),bearing_after_deg=math.degrees(after),
                center_inside_FOV_after=abs(after)<fov/2,
                actual_marker_size_m=target['size']))
        row['evaluator_annotations']=annotations
    return dict(evidence_type='prior_SLAM_raster_and_recorded_pose_near_pass_shadow_not_new_native_mission',
        episode=str(episode),snapshot=str(snapshot),workspace=str(workspace),rows=results,
        snapshot_elapsed_sim_s=elapsed,matched_exploration_journal=record,
        history_reset_for_each_probe=True,ground_truth_available_to_policy=False,
        evaluator_target_annotation_after_selection=True,native_result_modified=False,
        limits=['Previous decision raster and viewed state held fixed during one recorded exploration leg;not exact current live maps',
            'Camera/odom poses are interpolated from recorded audit samples;camera extrinsics inferred from accepted observations',
            'Each empty-history probe is independent;this does not simulate a new route,overhead,ID decoding or repeated-look behavior',
            'Target center in horizontal FOV does not prove unoccluded full-marker visibility or correct decoding'],
        source_sha256={str(path):digest(path) for path in [snapshot,trace,scene,episode/'evaluation.json',
            episode/'exploration_memory.jsonl',camera_diagnosis,module_path,Path(__file__)]})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('episode','snapshot','workspace','camera-diagnosis','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--start',type=float,required=True);parser.add_argument('--stop',type=float,required=True)
    args=parser.parse_args();result=replay(args.episode.resolve(),args.snapshot.resolve(),args.workspace.resolve(),args.camera_diagnosis.resolve(),args.start,args.stop)
    save(args.output,result);print(json.dumps(dict(probes=len(result['rows']),choices=sum(row['choice'] is not None for row in result['rows']))))
