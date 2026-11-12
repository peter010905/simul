"""One finite native probe of a new policy source in a fresh exterior space."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time

import yaml

from continual_native_curriculum import digest,save,source_hashes
from launch_spatial_curriculum import audit_geometry
from native_tournament import native_processes,service_active
from run_floor_pose_regression import floor_gate,probe
from runtime_configuration_v2 import probe as config_probe,configuration_gate
from spatial_native_curriculum_v5 import collect,fresh_scene

ROOT=Path(__file__).resolve().parents[1]


def prepare(args):
    output=args.output;output.mkdir(parents=True,exist_ok=False)
    (output/'scenarios').mkdir();case=output/'case_0';case.mkdir()
    cfg=fresh_scene(args.seed,args.obstacles,'mixed',args.shape)
    scene_file=output/'scenarios'/f'{args.seed}.yaml';scene_file.write_text(yaml.safe_dump(cfg,sort_keys=False))
    scene=dict(file=str(scene_file),seed=args.seed,sha256=digest(scene_file),mount_profile=cfg['mount_profile'],
        requested_obstacles=args.obstacles,actual_obstacles=len(cfg['obstacles']),
        arena={k:cfg['arena'][k] for k in ('shape','width_m','height_m','area_m2')})
    shutil.copy2(args.policy,case/'policy.json')
    save(case/'manifest.json',dict(environment='gazebo_harmonic',planner='edge',camera_width=960,duration=600,
        render_backend='software',start_docked=False,approach_radius_m=1.5,protocol_version=2,
        record_video=True,policy_sha256=digest(case/'policy.json'),scenarios=[scene]))
    state=dict(status='prepared_fresh_space_probe',phase='diagnostic',training_samples=False,
        generalization_verified=False,production_promoted=False,coordinator_unit=args.unit+'.service',
        source_hashes=source_hashes(args.workspace),jobs=[dict(number=0,phase='diagnostic',role='candidate',
            workspace=str(args.workspace),output=str(case),scenario=scene,status='prepared',domain=args.domain,
            parameters=json.loads((case/'policy.json').read_text())['parameters'],
            unit=args.unit+'-native.service')])
    save(output/'state.json',state);audit_geometry(output)


def run(args):
    if not (args.output/'state.json').exists():prepare(args)
    path=args.output/'state.json';state=json.loads(path.read_text());job=state['jobs'][0]
    while job['status']!='done':
        if source_hashes(args.workspace)!=state['source_hashes']:raise RuntimeError('Frozen probe runtime changed')
        directory=Path(job['output'])
        if job['status']=='prepared' and (directory/'service.json').exists():job['status']='running'
        if job['status']=='running' and not service_active(job['unit']):
            collect(job)
            if not (job.get('floor_sensor_gate') and job.get('runtime_configuration_gate')):
                job['raw_outcome_before_input_gates']=dict(job['outcome'])
                job['outcome'].update(success=False,infrastructure_failure=True,reason='native_inputs_unverified')
            state['status']='completed_fresh_space_probe';save(path,state);break
        if job['status']=='prepared' and len(native_processes())<2:
            result=subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],
                '--output',job['output'],'--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(job['domain']),'--record-video'],
                capture_output=True,text=True)
            (directory/'service_launch.log').write_text(result.stdout+result.stderr)
            if result.returncode:raise RuntimeError('Fresh-space probe native launch failed')
            job['status']='running';state['status']='running_fresh_space_probe';save(path,state)
        if job['status']=='running':
            live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if live.exists() and json.loads(live.read_text()).get('status',{}).get('state')=='running':
                if 'floor_sensor_gate' not in job:
                    job['floor_sensor_gate']=floor_gate(probe(job['domain'],directory/'floor_probe.json'));save(path,state)
                    if not job['floor_sensor_gate']:raise RuntimeError('Invalid native cliff input; preserve evidence')
                if not job.get('runtime_configuration_gate'):
                    report=config_probe(job['domain'],directory/'runtime_configuration.json')
                    check=configuration_gate(report,directory/'policy.json',1000)
                    save(directory/'runtime_configuration_check.json',check)
                    job['runtime_configuration_gate']=check['passed'];save(path,state)
                    if check['mismatches']:raise RuntimeError('Effective native profile mismatch')
        time.sleep(5)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('workspace','output','policy'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--seed',type=int,required=True)
    parser.add_argument('--shape',choices=['circle','triangle','trapezoid','rectangle','l_shape','u_shape','rooms'],required=True)
    parser.add_argument('--obstacles',type=int,required=True)
    parser.add_argument('--domain',type=int,default=118)
    parser.add_argument('--unit',required=True)
    args=parser.parse_args()
    for key in ('workspace','output','policy'):setattr(args,key,getattr(args,key).resolve())
    run(args)
