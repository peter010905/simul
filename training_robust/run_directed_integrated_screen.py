"""Screen the composed source on three known failures, then seven fresh spaces.

One diagnostic/training slot shares the host with the preserved CEM campaign.
No promotion or holdout consumption is performed by this engineering screen.
"""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

import yaml

from continual_native_curriculum import digest, save, source_hashes
from native_tournament import native_processes, service_active
from observe_spatial_campaign import native_pid
from probe_encoder_delta_profile import probe as velocity_probe, gate as velocity_gate
from run_floor_pose_regression import probe as floor_probe, floor_gate
from run_graph_landmark_regression import resume_cem
from run_near_pass_regression import inspect_slam_transport, wait_for_dependency
from run_windowed_witness_regression_v2 import settle_claims
from runtime_configuration_v2 import probe as config_probe, configuration_gate
from spatial_native_curriculum_v6 import benchmark_output_live, collect, observe, fresh_scene, quality

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-directed-integrated-v1.service'
SHAPES = ('circle', 'triangle', 'trapezoid', 'rectangle', 'l_shape', 'u_shape', 'rooms')


def validate_held_owner(held):
    state = json.loads((held/'state.json').read_text())
    if state['status'] != 'held_for_directed_marker_fix' or service_active(state['coordinator_unit']):
        raise RuntimeError('Old near-pass experiment must remain explicitly held')
    for job in state['jobs']:
        if (job['status'] != 'prepared' or service_active(job['unit'])
                or benchmark_output_live(job['output']) or (Path(job['output'])/'service.json').exists()):
            raise RuntimeError('Old held owner acquired a native claim; reconcile ownership first')


def profile_gates(result):
    look = result.get('marker_directed_look', {})
    near = result.get('near_pass_inspection', {})
    pixel = result.get('marker_focus_profile', {})
    graph = result.get('graph_landmark_update', {})
    return dict(
        directed_profile_gate=all(look.get(k) == v for k,v in dict(version=1,
            target_frame='odom',camera_axis='optical_z',creates_recognition_or_approach_credit=False,
            ground_truth_inputs=False).items()),
        near_pass_profile_gate=all(near.get(k) == v for k,v in dict(maximum_sim_s=35.,
            maximum_looks=8,maximum_camera_range_m=1.2).items()),
        pixel_profile_gate=all(pixel.get(k) == v for k,v in dict(desired_edge_pixels=36.,
            uncertain_view_penalty_s=8.).items()),
        graph_profile_gate=graph.get('ground_truth_inputs') is False)


def prepare(parent, dependency, held, workspace, output):
    validate_held_owner(held)
    test_log = (workspace.parent/'build_and_tests.log').read_text()
    if '337 passed' not in test_log or '1 package finished' not in test_log:
        raise ValueError('Require the full runtime tests and completed installed build')
    integrated = ROOT/'training_robust/native90/integrated_v1/runtime'
    before, after = source_hashes(integrated), source_hashes(workspace)
    changed = {p for p in before.keys()|after.keys() if before.get(p) != after.get(p)}
    if changed != {'osp_mission/mission.py','osp_mission/marker_directed_look.py','test/test_marker_directed_runtime.py'}:
        raise ValueError(f'Unexpected changes to the tested integrated source: {changed}')
    parent_state = json.loads((parent/'state.json').read_text())
    base = source_hashes(Path(json.loads((parent/'configuration.json').read_text())['workspace']))
    protected = ('osp_mission/detector.py','osp_mission/marker_approach.py','config/sim_ekf.yaml',
                 'osp_mission/space_geometry.py','config/nav2_gazebo.yaml','launch/simulation.launch.py')
    if any(base[p] != after[p] for p in protected):
        raise ValueError('Recognition, physical approach, navigation or world geometry gates changed')
    output.mkdir(parents=True,exist_ok=False)
    (output/'scenarios').mkdir()
    jobs = []
    for index in (3,4,6):
        reference = parent_state['jobs'][index]
        if reference['status'] != 'done' or reference['outcome']['success']:
            raise ValueError('Require the preserved rectangle, L and rooms failures')
        jobs.append(dict(scenario=reference['scenario'],parameters=reference['parameters'],
            phase='diagnostic',baseline=reference['outcome'],baseline_output=reference['output']))
    used = {j['scenario'].get('physical_sha256') for j in parent_state['jobs']}
    weights = parent_state['jobs'][0]['parameters']
    for i,shape in enumerate(SHAPES):
        difficulty = (8,12,16,24,32)[i%5]
        cfg = fresh_scene(1191500000+i*101,difficulty,('walls','mixed','obstacles','hidden_obstacles')[i%4],shape)
        path = output/'scenarios'/f'{cfg["seed"]}.yaml'
        path.write_text(yaml.safe_dump(cfg,sort_keys=False))
        physical = {k:cfg[k] for k in ('arena','start','obstacles','targets') if k in cfg}
        fingerprint = hashlib.sha256(json.dumps(physical,sort_keys=True).encode()).hexdigest()
        if fingerprint in used:raise ValueError('Fresh training map already used')
        used.add(fingerprint)
        scene = dict(file=str(path),seed=cfg['seed'],sha256=digest(path),physical_sha256=fingerprint,
            requested_obstacles=difficulty,actual_obstacles=len(cfg['obstacles']),mount_profile=cfg['mount_profile'],
            arena={k:cfg['arena'][k] for k in ('shape','width_m','height_m','area_m2')})
        jobs.append(dict(scenario=scene,parameters=weights,phase='train'))
    for number,job in enumerate(jobs):
        directory=output/f'case_{number}';directory.mkdir()
        policy=dict(planner='edge',parameters=job['parameters'],status='unqualified_integrated_candidate')
        save(directory/'policy.json',policy)
        save(directory/'manifest.json',dict(environment='gazebo_harmonic',planner='edge',camera_width=960,
            duration=600,render_backend='software',start_docked=False,approach_radius_m=1.5,
            protocol_version=2,record_video=True,policy_sha256=digest(directory/'policy.json'),scenarios=[job['scenario']]))
        job.update(number=number,generation=0,role='candidate',workspace=str(workspace),output=str(directory),
                   status='prepared',domain=131+number%3,unit=f'osp-directed-integrated-v1-{number:02d}.service')
    drivers=[Path(__file__),ROOT/'scripts/prepare_marker_directed_runtime.py',
        ROOT/'scripts/candidate_marker_directed_look.py',ROOT/'scripts/prepare_integrated_runtime.py',
        ROOT/'training_robust/run_near_pass_regression.py',ROOT/'training_robust/run_graph_landmark_regression.py',
        ROOT/'training_robust/run_windowed_witness_regression_v2.py',ROOT/'training_robust/probe_encoder_delta_profile.py',
        ROOT/'training_robust/spatial_native_curriculum_v6.py',ROOT/'training_robust/run_native_service.py']
    save(output/'state.json',dict(status='running',phase='engineering_screen',jobs=jobs,
        coordinator_unit=UNIT,parent_campaign=str(parent),dependency=str(dependency),held_owner=str(held),
        source_hashes=after,driver_hashes={str(p):digest(p) for p in drivers},
        protected_source_files=list(protected),runtime_tests=337,
        production_promoted=False,generalization_verified=False,holdout_consumed=False,
        comparison_limit='Three repeated failure scenes are diagnostic only. Seven subsequent physical worlds are fresh training screens with fixed weights, not independent qualification. Motion source, nearby looks, pixel ranking, SLAM transport, final graph projection and directed looks differ from S7; timing differences cannot be assigned to a single change.'))


def collect_screen(job):
    collect(job)
    directory=Path(job['output'])
    report=json.loads((directory/'report.json').read_text()) if (directory/'report.json').exists() else {}
    result=report.get('rows',[{}])[0].get('result',{}) if report.get('rows') else {}
    job.update(profile_gates(result))
    for key in ('marker_directed_look','near_pass_inspection','graph_landmark_update'):
        job[key]=result.get(key,{})
    gates=('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate','slam_transport_gate',
           'directed_profile_gate','near_pass_profile_gate','pixel_profile_gate','graph_profile_gate')
    if not all(job.get(k) is True for k in gates):
        job['raw_outcome_before_integrated_gates']=dict(job['outcome'])
        job['outcome'].update(success=False,infrastructure_failure=True,reason='integrated_effective_inputs_unverified')


def measure(job,pid):
    directory=Path(job['output'])
    live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
    if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state')!='running':return
    if not job.get('slam_transport_gate'):
        measured=inspect_slam_transport(pid) if pid is not None else {}
        save(directory/'effective_slam_transport.json',measured)
        job['slam_transport_gate']=bool(measured.get('server_environment',{}).get('FASTDDS_BUILTIN_TRANSPORTS')=='UDPv4' and measured.get('shm_threads')==[])
    if not job.get('floor_sensor_gate'):
        report=observe(floor_probe,job,'floor_probe.json')
        if report is not None and floor_gate(report):job['floor_sensor_gate']=True
    if job.get('floor_sensor_gate') and not job.get('runtime_configuration_gate'):
        report=observe(config_probe,job,'runtime_configuration.json')
        if report is not None:
            check=configuration_gate(report,directory/'policy.json',1000)
            save(directory/'runtime_configuration_check.json',check)
            if check['mismatches']:raise RuntimeError('Installed navigation or detector parameters differ')
            job['runtime_configuration_gate']=check['passed']
    if job.get('runtime_configuration_gate') and not job.get('velocity_profile_gate'):
        report=observe(velocity_probe,job,'effective_velocity_profile.json')
        if report is not None:
            job['velocity_profile_gate']=velocity_gate(report)
            if report.get('complete') and not job['velocity_profile_gate']:
                raise RuntimeError('Installed encoder velocity source differs')


def run(parent,dependency,held,workspace,output):
    path=output/'state.json'
    if not path.exists():prepare(parent,dependency,held,workspace,output)
    lock=(output/'coordinator.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    state=json.loads(path.read_text())
    if all(j['status']=='done' for j in state['jobs']):return
    wait_for_dependency(dependency,output)
    for job in state['jobs']:
        if job['status']=='done':continue
        validate_held_owner(held)
        owner=json.loads((parent/'state.json').read_text())
        if owner['status']!='running':raise RuntimeError('Parent campaign requires collection or repair first')
        restored=False
        try:
            # Stop only the dispatcher. Existing Gazebo units are independent.
            if service_active(owner['coordinator_unit']):
                subprocess.run(['systemctl','--user','stop',owner['coordinator_unit']],check=True)
            settle_claims(parent,dependency,output)
            while job['status']!='done':
                if source_hashes(workspace)!=state['source_hashes']:raise RuntimeError('Frozen integrated runtime changed')
                if any(digest(Path(p))!=h for p,h in state['driver_hashes'].items()):
                    raise RuntimeError('Frozen integrated driver changed')
                directory=Path(job['output'])
                if job['status']=='prepared' and (directory/'service.json').exists():job['status']='running'
                try:
                    pid=native_pid(job['unit'],directory) if job['status']=='running' else None
                    if pid and not restored:
                        resume_cem(parent);restored=True
                        save(directory/'dispatch_handoff.json',dict(verified_native_pid=pid,
                            resumed_unit=owner['coordinator_unit'],original_native_runs_preserved=True))
                    if job['status']=='running' and not service_active(job['unit']) and not benchmark_output_live(directory):
                        time.sleep(2)
                        collect_screen(job);save(path,state)
                        print(json.dumps(dict(number=job['number'],phase=job['phase'],**job['outcome'])),flush=True)
                        break
                    if job['status']=='prepared' and len(native_processes())<2:
                        result=subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                            '--unit',job['unit'].removesuffix('.service'),'--workspace',str(workspace),'--output',str(directory),
                            '--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                            '--output',str(directory),'--run-prepared','--domain',str(job['domain']),'--record-video'],
                            capture_output=True,text=True)
                        (directory/'service_launch.log').write_text(result.stdout+result.stderr)
                        if result.returncode:raise RuntimeError('Integrated native launch failed; preserve service metadata')
                        job['status']='running';save(path,state)
                    if job['status']=='running':measure(job,pid)
                    save(path,state)
                except subprocess.TimeoutExpired as exc:
                    save(directory/'observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
                time.sleep(3)
        finally:
            settle_claims(parent,dependency,output)
            resume_cem(parent)
    state['status']='completed_integrated_screen'
    state['results']={phase:quality([j['outcome'] for j in state['jobs'] if j['phase']==phase])
                     for phase in ('diagnostic','train')}
    save(path,state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','dependency','held','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    paths={name:getattr(args,name).resolve() for name in ('parent','dependency','held','workspace','output')}
    if args.prepare_only:prepare(**paths)
    else:run(**paths)
