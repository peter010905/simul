"""Paired native crowded-start regression; truth is used only by the evaluator."""
import argparse
import csv
import fcntl
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.ndimage import distance_transform_edt,label
import yaml

from tb4_platform_curriculum import digest,save,source_hashes,occupied_slots,benchmark_output_live
from native_tournament import native_processes,service_active
from tb4_platform_probe import probe,configuration_gate,motion_gate,transfer_gate
from run_floor_pose_regression import probe as floor_probe,floor_gate

ROOT=Path(__file__).resolve().parents[1]


def scene(seed,width=.55,yaw=0.):
    sys.path.insert(0,str(ROOT/'src/osp_mission'))
    from osp_mission.space_geometry import wall,rasterize_space
    polygon=[[0.,0.],[14.,0.],[14.,10.],[0.,10.]]
    arena=dict(schema_version=1,shape='rectangle',width_m=14.,height_m=10.,polygon=polygon,
        walls=[wall(a,b,boundary=True) for a,b in zip(polygon,polygon[1:]+polygon[:1])],
        doorways=[],area_m2=140.,pdf_fixtures=False)
    def box(x,y,sx,sy):return dict(type='box',kind='rack',x=x,y=y,sx=sx,sy=sy,height=.9,yaw=0.)
    obstacles=[box(6.,5.-width/2-.225,4.,.45),box(6.,5.+width/2+.225,4.,.45),box(3.85,5.,.3,width+.9)]
    obstacles += [box(x,y,.8,.8) for x,y in [(10.,2.),(10.,5.),(10.,8.),(4.,2.),(4.,8.),(8.,8.),(8.,2.),(12.,3.),(12.,7.)]]
    targets=[dict(id=ident,x=obstacles[i]['x']+.455,y=obstacles[i]['y'],yaw=0.,size=.18,
                  mount='obstacle',carrier_obstacle=i,hidden_from_start=True) for ident,i in zip([106,246,477],[3,4,5])]
    cfg=dict(seed=seed,randomize=False,task='crowded_start_regression',arena=arena,obstacles=obstacles,targets=targets,
        start=[4.6,5.,yaw],difficulty=len(obstacles),actual_obstacles=len(obstacles),mount_profile='hidden_obstacles',
        regression=dict(corridor_width_m=width,exit_world_x=8.3,scope='startup/escape only, not complete QR task qualification',ground_truth_given_to_policy=False))
    resolution=.025;occupied=rasterize_space(cfg,resolution)
    clear=distance_transform_edt(np.pad(~occupied,1,constant_values=False))[1:-1,1:-1]*resolution
    body=clear>=.189+resolution/math.sqrt(2)+.01;groups,_=label(body)
    sy,sx=int(cfg['start'][1]/resolution),int(cfg['start'][0]/resolution)
    assert groups[sy,sx] and groups[sy,sx]==groups[int(5./resolution),int(8.5/resolution)]
    for target in targets:
        assert groups[int(target['y']/resolution),int((target['x']+.85)/resolution)]==groups[sy,sx]
    return cfg,dict(start_clearance_m=float(clear[sy,sx]),body_clearance_required_m=.189+resolution/math.sqrt(2)+.01,
                   exit_reachable=True,marker_approaches_reachable=True,evidence='offline geometry, not native success')


def prepare(root,candidate,baseline):
    root.mkdir(parents=True,exist_ok=False);jobs=[]
    for case,(width,yaw) in enumerate([(.55,0.),(.60,math.pi/2)]):
        cfg,geometry=scene(1196000000+case,width,yaw)
        file=root/f'scene_{case}.yaml';file.write_text(yaml.safe_dump(cfg,sort_keys=False))
        for role,workspace in [('baseline',baseline),('candidate',candidate)]:
            number=len(jobs);directory=root/f'{number:02d}_{role}';directory.mkdir()
            (directory/'policy.json').write_bytes((ROOT/'policy_releases/2026-11-13_face_intent_v2/policy.json').read_bytes())
            entry=dict(file=str(file),seed=cfg['seed'],sha256=digest(file),arena=cfg['arena'],mount_profile='hidden_obstacles',
                       actual_obstacles=len(cfg['obstacles']),requested_obstacles=len(cfg['obstacles']))
            save(directory/'manifest.json',dict(environment='gazebo_harmonic',planner='edge',camera_width=250,duration=120.,
                render_backend='software',start_docked=False,approach_radius_m=1.5,protocol_version=2,
                policy_sha256=digest(directory/'policy.json'),scenarios=[entry],purpose='paired crowded-start regression, not full task qualification'))
            jobs.append(dict(number=number,case=case,phase='regression',generation=0,role=role,workspace=str(workspace),
                output=str(directory),scenario=entry,status='prepared',unit=f'osp-crowded-start-v1-{number:02d}.service',
                geometry=geometry,stress_profile='none'))
    save(root/'configuration.json',dict(source_hashes={str(w):source_hashes(w) for w in [candidate,baseline]},
        driver_sha256=digest(Path(__file__).resolve()),native_limit=2,duration=120.,hardware_validated=False,
        support_hashes={str(p):digest(p) for p in [ROOT/'training_robust'/name for name in
            ['tb4_platform_curriculum.py','tb4_platform_probe.py','runtime_configuration_v2.py',
             'native_tournament.py','run_floor_pose_regression.py','run_native_service.py','gazebo_benchmark.py']]},
        prepared_hashes={str(p):digest(p) for p in list(root.glob('scene_*.yaml'))+
            list(root.glob('*/manifest.json'))+list(root.glob('*/policy.json'))}))
    save(root/'state.json',dict(status='waiting_for_native_slots',generation=0,phase='regression',jobs=jobs,
        coordinator_unit='osp-crowded-start-regression-v1.service'))


def observe_job(job):
    directory=Path(job['output']);episode=directory/'episodes'/str(job['scenario']['seed'])
    if not (episode/'live_status.json').exists():return
    if not all(job.get(k) for k in ['configuration_gate','platform_gate','transfer_gate']):
        try:
            report=probe(job['domain'],directory/'runtime_configuration.json')
            checked=configuration_gate(report,directory/'policy.json',expected_duration=120.)
            save(directory/'runtime_configuration_check.json',checked)
            job.update(configuration_gate=checked['passed'],platform_gate=motion_gate(report),transfer_gate=transfer_gate(report,'none'))
        except (OSError,ValueError,subprocess.SubprocessError) as exc:job['observation_error']=repr(exc)
    if job.get('platform_gate') and not job.get('floor_sensor_gate'):
        try:job['floor_sensor_gate']=floor_gate(floor_probe(job['domain'],directory/'floor_probe.json'))
        except (OSError,ValueError,subprocess.SubprocessError) as exc:job['floor_observation_error']=repr(exc)


def collect(job):
    directory=Path(job['output']);episode=directory/'episodes'/str(job['scenario']['seed']);path=episode/'evaluation.json'
    result=json.loads(path.read_text()) if path.exists() else {}
    trace=episode/'ground_truth_trajectory.csv';rows=list(csv.DictReader(trace.open())) if trace.exists() else []
    first=float(rows[0]['sim_seconds']) if rows else 0.
    exit_time=next((float(r['sim_seconds'])-first for r in rows if float(r['world_x'])>=8.3),None)
    physical_path=sum(math.hypot(float(b['world_x'])-float(a['world_x']),float(b['world_y'])-float(a['world_y'])) for a,b in zip(rows,rows[1:]))
    gates=all(job.get(k) is True for k in ['configuration_gate','platform_gate','transfer_gate','floor_sensor_gate'])
    completed=result.get('reason') in ('timeout','all_targets_found')
    job.update(status='done',outcome=dict(escaped=bool(gates and completed and exit_time is not None
            and 0 <= exit_time <= 120. and result.get('collisions')==0),
        exit_seconds=exit_time,actual_path_m=physical_path,contacts=result.get('collisions'),reason=result.get('reason'),
        valid_input=gates,completed_normally=completed,
        full_task_success=result.get('success',False),qualification_scope='crowded-start only',
        exit_timing_origin='first independent ground-truth sample; includes pre-mission startup'))
    save(directory/'crowded_start_audit.json',job['outcome'])


def run(root):
    lock=(root/'coordinator.lock').open('w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    config=json.loads((root/'configuration.json').read_text());state=json.loads((root/'state.json').read_text())
    while any(j['status']!='done' for j in state['jobs']):
        for workspace,expected in config['source_hashes'].items():
            if source_hashes(workspace)!=expected:raise RuntimeError('Frozen runtime changed')
        if digest(Path(__file__).resolve())!=config['driver_sha256']:raise RuntimeError('Frozen driver changed')
        for path,expected in {**config['support_hashes'],**config['prepared_hashes']}.items():
            if digest(Path(path))!=expected:raise RuntimeError(f'Frozen experiment input changed: {path}')
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():
                job['status']='running'
        for job in [j for j in state['jobs'] if j['status']=='running']:
            if service_active(job['unit']):observe_job(job)
            elif not benchmark_output_live(job['output']):collect(job)
        active=[j for j in state['jobs'] if j['status']=='running']
        occupied=occupied_slots(state['jobs']);domains={j['domain'] for j in active}
        for job in [j for j in state['jobs'] if j['status']=='prepared'][:max(0,2-occupied)]:
            if len(native_processes())>=2:break
            job['domain']=next(d for d in [171,172] if d not in domains);domains.add(job['domain'])
            save(root/'state.json',state)
            command=['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                '--unit',job['unit'].removesuffix('.service'),'--workspace',job['workspace'],'--output',job['output'],'--',
                '/usr/bin/env','OSP_TRANSFER_PROFILE=none','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                '--output',job['output'],'--run-prepared','--domain',str(job['domain']),'--record-video']
            subprocess.run(command,check=True,capture_output=True,text=True);job['status']='running'
            save(root/'state.json',state)
        state['status']='running' if any(j['status']=='running' for j in state['jobs']) else 'waiting_for_native_slots'
        save(root/'state.json',state);time.sleep(5)
    state['status']='completed';save(root/'state.json',state)
    print(json.dumps([j['outcome'] for j in state['jobs']]),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--candidate',type=Path);p.add_argument('--baseline',type=Path);p.add_argument('--run-prepared',action='store_true')
    a=p.parse_args();root=a.output.resolve()
    if not a.run_prepared:prepare(root,a.candidate.resolve(),a.baseline.resolve())
    run(root)
