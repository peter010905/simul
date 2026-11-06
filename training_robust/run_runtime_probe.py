"""Run one prepared Gazebo scene to measure actual sensor and motion timing."""

import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path

from osp_mission.gazebo_eval import one


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--prepared',default='training_robust/gazebo_runtime_calibration')
    parser.add_argument('--domain',type=int,default=84)
    parser.add_argument('--wall-timeout',type=float,default=5400.)
    parser.add_argument('--record-video',action='store_true',help='Record Gazebo cameras and live LiDAR/SLAM to video/mission.mp4')
    parser.add_argument('--scene-index',type=int,default=0,help='Index in the prepared 20-scene manifest')
    args=parser.parse_args();prepared=Path(args.prepared).resolve()
    manifest=json.loads((prepared/'manifest.json').read_text())
    if not 0<=args.scene_index<len(manifest['scenarios']):parser.error('--scene-index is outside the prepared manifest')
    entry=manifest['scenarios'][args.scene_index]
    policy=prepared/'policy.json';scenario=Path(entry['file'])
    if hashlib.sha256(policy.read_bytes()).hexdigest()!=manifest['policy_sha256']:
        raise ValueError('Prepared policy changed')
    if hashlib.sha256(scenario.read_bytes()).hexdigest()!=entry['sha256']:
        raise ValueError('Prepared scenario changed')
    out=prepared/'runtime_probes'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    out.mkdir(parents=True,exist_ok=False)
    files=list(Path('src/osp_mission/osp_mission').glob('*.py'))+list(Path('src/osp_mission/config').glob('*.yaml'))+list(Path('src/osp_mission/launch').glob('*.py'))
    (out/'probe_configuration.json').write_text(json.dumps(dict(scenario=entry,domain=args.domain,
        record_video=args.record_video,render_backend=manifest.get('render_backend','software'),start_docked=manifest.get('start_docked',False),
        source_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        purpose='Measure actual Gazebo timing before further surrogate tuning; a single scene is not a robustness benchmark'),indent=2))
    print(json.dumps({'starting_probe':str(out),'scene_index':args.scene_index,
        'obstacles':entry['actual_obstacles'],'mount_profile':entry['mount_profile'],
        'record_video':args.record_video}),flush=True)
    result=one(scenario,policy,out,domain=args.domain,duration=manifest['duration'],
               wall_timeout=args.wall_timeout,planner=manifest['planner'],camera_width=manifest['camera_width'],record_video=args.record_video,render_backend=manifest.get('render_backend','software'),start_docked=manifest.get('start_docked',False))
    metrics=result.get('runtime_metrics',{})
    usable=bool(metrics.get('complete_sensor_coverage') and metrics.get('stationary_turn_rad_s',{}).get('samples',0)>=10
                and metrics.get('moving_speed_m_s',{}).get('samples',0)>=10)
    report=dict(output=str(out),mission_success=result['success'],reason=result['reason'],
                timing_measurements_available=usable,runtime_metrics=metrics)
    if args.record_video:report['recording']=result.get('recording',{'video':None,'error':'Simulation did not start'})
    (out/'probe_summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
    if not usable:raise SystemExit(2)


if __name__=='__main__':main()
