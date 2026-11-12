"""Evaluator-only camera-direction diagnosis at recorded processed image stamps."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
import yaml

from continual_native_curriculum import digest,save


def values(path,keys):
    return np.array([[float(row[k]) for k in keys] for row in csv.DictReader(path.open())])


def analyze(episode,scene,calibration):
    camera=json.loads(calibration.read_text());cfg=yaml.safe_load(scene.read_text())
    if not camera['native_identity_verified_after_observation']:raise ValueError('Require verified CameraInfo')
    poses=values(episode/'pose_trace.csv',['map_stamp','map_x','map_y','map_yaw'])
    poses=poses[np.r_[True,np.diff(poses[:,0])>0]];poses[:,3]=np.unwrap(poses[:,3])
    observations=json.loads((episode/'marker_observations.json').read_text());offsets=[];angles=[]
    if not observations:raise ValueError('Cannot infer camera extrinsics without recorded observations')
    for o in observations:
        m=np.array([np.interp(o['stamp'],poses[:,0],poses[:,k]) for k in [1,2,3]])
        delta=np.array(o['camera_origin_map'])[:2]-m[:2];c,s=math.cos(m[2]),math.sin(m[2])
        offsets.append([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
        delta=np.array(o['map_point'])[:2]-np.array(o['camera_origin_map'])[:2];point=o['camera_point']
        angle=math.atan2(delta[1],delta[0])-math.atan2(-point[0],point[2])-m[2]
        angles.append((angle+math.pi)%(2*math.pi)-math.pi)
    offset=np.median(offsets,axis=0);yaw_offset=float(np.median(angles))
    fine=[]
    for path in episode.glob('evaluator_gt_fine*.csv'):
        meta=path.with_suffix('.json')
        if meta.exists() and json.loads(meta.read_text()).get('complete'):
            samples=values(path,['sim_seconds','world_x','world_y','world_yaw'])
            if len(samples):fine.append((path,samples))
    if not fine:raise ValueError('Require completed independent fine trajectory')
    truth=np.concatenate([row for _,row in fine]);truth=truth[np.argsort(truth[:,0])]
    truth[:,3]=np.unwrap(truth[:,3]);times=np.array([float(row['stamp_sim_seconds'])
        for row in csv.DictReader((episode/'runtime_trace.csv').open()) if row['stream']=='camera'])
    times=times[(times>=truth[0,0])&(times<=truth[-1,0])]
    base=np.array([np.interp(times,truth[:,0],truth[:,k]) for k in [1,2,3]]).T
    position=base[:,:2]+np.column_stack([np.cos(base[:,2])*offset[0]-np.sin(base[:,2])*offset[1],
        np.sin(base[:,2])*offset[0]+np.cos(base[:,2])*offset[1]])
    heading=base[:,2]+yaw_offset;focal=camera['k'][0];half_fov=camera['horizontal_fov_rad']/2
    evaluation=json.loads((episode/'evaluation.json').read_text());rows=[]
    for target in cfg['targets']:
        delta=np.array([target['x'],target['y']])-position;distance=np.linalg.norm(delta,axis=1)
        bearing=(np.arctan2(delta[:,1],delta[:,0])-heading+math.pi)%(2*math.pi)-math.pi
        incidence=(-delta[:,0]*math.cos(target['yaw'])-delta[:,1]*math.sin(target['yaw']))/distance
        pixels=focal*target['size']*np.maximum(0,incidence)/distance
        front=incidence>0;in_fov=abs(bearing)<half_fov;readable=front&(pixels>=18)
        nearest=int(np.argmin(distance));fov_pixels=pixels[front&in_fov]
        rows.append(dict(id=target['id'],decoded_frames=evaluation['detector']['decoded'].get(str(target['id']),0),
            modeled_readable_in_fov_frames_before_occlusion=int((readable&in_fov).sum()),
            maximum_predicted_pixels_in_front_fov=float(fov_pixels.max()) if len(fov_pixels) else None,
            minimum_bearing_error_deg_when_predicted_readable=float(np.degrees(abs(bearing[readable]).min())) if readable.any() else None,
            closest_camera_distance_m=float(distance[nearest]),
            bearing_error_deg_at_closest=float(np.degrees(bearing[nearest]))))
    sources=[scene,calibration,episode/'pose_trace.csv',episode/'marker_observations.json',
        episode/'runtime_trace.csv',episode/'evaluation.json',*[path for path,_ in fine]]
    return dict(evidence_type='offline_2D_camera_geometry_at_recorded_image_stamps',episode=str(episode),rows=rows,
        inferred_camera_extrinsic_xy_m=offset.tolist(),inferred_camera_yaw_offset_rad=yaw_offset,
        actual_horizontal_half_fov_deg=math.degrees(half_fov),source_sha256={str(path):digest(path) for path in sources},
        ground_truth_used_only_by_evaluator=True,policy_or_result_modified=False,
        limits=['Pinhole edge size is approximate;occlusion,3D corner projection and image decoder behavior are not simulated',
            'Camera extrinsics are inferred from accepted observations and interpolated audit poses',
            'This diagnoses missed viewing direction;it is not a counterfactual mission success or speed result'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['episode','scene','calibration','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();save(args.output,analyze(args.episode.resolve(),args.scene.resolve(),args.calibration.resolve()))
