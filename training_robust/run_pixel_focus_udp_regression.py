"""Retry the terminal startup failure with SLAM-only UDP transport in a new stage."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time

from continual_native_curriculum import digest,save,source_hashes
from native_tournament import native_processes,service_active
from observe_spatial_campaign import native_pid
from probe_velocity_profile import probe as velocity_probe,gate as velocity_gate
from run_floor_pose_regression import probe as floor_probe,floor_gate
from run_graph_landmark_regression import resume_cem
from run_windowed_witness_regression_v2 import settle_claims
from runtime_configuration_v2 import probe as config_probe,configuration_gate
from spatial_native_curriculum_v6 import benchmark_output_live,collect,observe
from capture_native_failures import descendants

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-pixel-focus-udp-v1.service'


def inspect_slam_transport(native):
    table={}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:table[int(path.name)]=int((path/'stat').read_text().rpartition(')')[2].split()[1])
        except (OSError,ValueError,IndexError):pass
    for pid in descendants(native,table):
        path=Path('/proc')/str(pid)
        try:argv=(path/'cmdline').read_bytes().decode().split('\0')
        except OSError:continue
        if not any(value.endswith('/async_slam_toolbox_node') for value in argv):continue
        try:
            environment=dict(row.split('=',1) for row in (path/'environ').read_bytes().decode().split('\0') if '=' in row)
            names=[]
            for task in (path/'task').iterdir():
                try:names.append((task/'comm').read_text().strip())
                except OSError:pass
            return dict(native_pid=native,server_pid=pid,
                server_environment={key:environment[key] for key in ('ROS_DOMAIN_ID','FASTDDS_BUILTIN_TRANSPORTS') if key in environment},
                shm_threads=[name for name in names if 'dds.shm' in name],source='Read-only bound native SLAM process environment and thread names')
        except OSError:continue
    return dict(native_pid=native,reason='SLAM_process_not_observed')

def validate_previous_failure(dependency):
    state=json.loads((dependency/'state.json').read_text());job=state['jobs'][0]
    if (job['status']!='done' or service_active(state['coordinator_unit'])
            or service_active(job['unit']) or benchmark_output_live(job['output'])):
        raise RuntimeError('Require actual terminal prior startup attempt,not an observation timeout')
    evaluation=Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'evaluation.json'
    report=json.loads(evaluation.read_text())
    if report.get('reason')!='startup_failure':raise ValueError('Only the preserved real startup failure authorizes this transport ablation')
    return job,dict(root=str(dependency),outcome=job['outcome'],native_reason=report['reason'],previous_attempt_retained=True)


def prepare(parent,dependency,workspace,output):
    previous,failure=validate_previous_failure(dependency)
    reference=json.loads((parent/'state.json').read_text())['jobs'][1]
    if reference['status']!='done' or not reference['outcome']['success']:
        raise ValueError('Require the finalized375.1s native triangle baseline')
    if '270 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require full candidate runtime tests and completed build')
    base=Path(json.loads((parent/'configuration.json').read_text())['workspace'])
    before,after=source_hashes(base),source_hashes(workspace)
    changed={p for p in before.keys()|after.keys() if before.get(p)!=after.get(p)}
    expected={'osp_mission/marker_focus.py','osp_mission/mission.py','test/test_pixel_focus_runtime.py','launch/robot.launch.py'}
    if changed!=expected:raise ValueError(f'Unexpected source changes:{changed}')
    original=source_hashes(Path(previous['workspace']))
    transport_only={p for p in original.keys()|after.keys() if original.get(p)!=after.get(p)}
    if transport_only!={'launch/robot.launch.py'}:raise ValueError('Policy must remain identical to failed attempt;only scoped SLAM transport changes')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(reference['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(reference['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    state=dict(status='running',phase='diagnostic',training_samples=False,
        generalization_verified=False,production_promoted=False,parent_campaign=str(parent),
        coordinator_unit=UNIT,source_hashes=after,source_changes=sorted(changed),baseline=reference['outcome'],
        prior_startup_failure=failure,transport_only_changes_vs_failed_attempt=sorted(transport_only),
        comparison_limit='Same triangle/weights and pixel-aware policy;SLAM-only transport changes to UDPv4 after a preserved terminal startup failure. Performance is not an isolated focus-cost comparison against S7.',
        driver_hashes={str(p):digest(p) for p in [Path(__file__),ROOT/'scripts/prepare_pixel_focus_runtime.py',
            ROOT/'training_robust/replay_pixel_focus.py',ROOT/'training_robust/run_graph_landmark_regression.py',
            ROOT/'scripts/prepare_slam_rpc_transport_runtime.py',ROOT/'training_robust/probe_slam_state_transport.py']},
        jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',workspace=str(workspace),
            output=str(case),scenario=reference['scenario'],parameters=reference['parameters'],
            status='prepared',domain=128,unit=UNIT.removesuffix('.service')+'-native.service')])
    save(output/'state.json',state)


def wait_for_dependency(dependency,output):
    """Do not race the preceding owner's final CEM-resume callback."""
    while True:
        state=json.loads((dependency/'state.json').read_text());live=[]
        try:
            owner_active=service_active(state['coordinator_unit'])
            for job in state['jobs']:
                pid=native_pid(job['unit'],Path(job['output'])) if job['status']=='running' else None
                if pid is not None or benchmark_output_live(job['output']):live.append(dict(unit=job['unit'],pid=pid))
            save(output/'dependency_wait.json',dict(coordinator_unit=state['coordinator_unit'],
                coordinator_active=owner_active,verified_native_handles=live,
                dependent_native_not_started=True,parent_dispatch_not_stopped=True))
            if not owner_active and not live:
                if not all(job['status']=='done' for job in state['jobs']):
                    raise RuntimeError('Dependency is terminal without collected native outcome;repair its collection before dispatch')
                return
        except subprocess.TimeoutExpired as exc:
            save(output/'dependency_observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
        time.sleep(3)


def run(parent,dependency,workspace,output):
    path=output/'state.json'
    if not path.exists():prepare(parent,dependency,workspace,output)
    state=json.loads(path.read_text());job=state['jobs'][0]
    if job['status']=='done':return
    owner=json.loads((parent/'state.json').read_text())
    if owner['status']!='running':raise RuntimeError('Require an active CEM campaign')
    if job['status']=='prepared':wait_for_dependency(dependency,output)
    owner=json.loads((parent/'state.json').read_text())
    if owner['status']!='running':raise RuntimeError('Parent changed to terminal or repair-needed state while waiting')
    restored=False
    try:
        if service_active(owner['coordinator_unit']):
            subprocess.run(['systemctl','--user','stop',owner['coordinator_unit']],check=True)
        settle_claims(parent,dependency,output)
        while job['status']!='done':
            if source_hashes(workspace)!=state['source_hashes']:raise RuntimeError('Frozen pixel focus runtime changed')
            if any(digest(Path(p))!=expected for p,expected in state['driver_hashes'].items()):
                raise RuntimeError('Frozen pixel focus driver changed')
            directory=Path(job['output'])
            if job['status']=='prepared' and (directory/'service.json').exists():job['status']='running'
            try:pid=native_pid(job['unit'],directory) if job['status']=='running' else None
            except subprocess.TimeoutExpired as exc:
                save(output/'native_pid_observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
                time.sleep(3);continue
            if pid and not restored:
                resume_cem(parent);restored=True
                save(output/'dispatch_handoff.json',dict(verified_new_native_pid=pid,
                    resumed_unit=owner['coordinator_unit'],driver='spatial_native_curriculum_v6.py',
                    old_native_runs_and_prepared_scenes_preserved=True))
            if job['status']=='running' and not service_active(job['unit']) and not benchmark_output_live(directory):
                # Allow the read-only evaluator recorder to finalize its last buffered trace.
                time.sleep(2)
                collect(job)
                if not all(job.get(k) is True for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate','slam_transport_gate')):
                    job['raw_outcome_before_input_gates']=dict(job['outcome'])
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='native_effective_inputs_unverified')
                result=json.loads((directory/'report.json').read_text())['rows'][0]['result'] if (directory/'report.json').exists() else {}
                job['pixel_focus_profile_gate']=result.get('marker_focus_profile')==dict(desired_edge_pixels=36.,uncertain_view_penalty_s=8.)
                episode=directory/'episodes'/str(job['scenario']['seed'])
                records=episode/'marker_followups.jsonl'
                job['pixel_aware_followups']=[json.loads(row) for row in records.read_text().splitlines()
                    if json.loads(row).get('predicted_min_edge_pixels') is not None] if records.exists() else []
                if job['outcome']['success'] and not job['pixel_focus_profile_gate']:
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='native_pixel_focus_profile_not_observed')
                state['status']='completed_pixel_focus_udp_regression';save(path,state);break
            if job['status']=='prepared' and len(native_processes())<2:
                result=subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                    '--unit',job['unit'].removesuffix('.service'),'--workspace',str(workspace),'--output',str(directory),
                    '--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                    '--output',str(directory),'--run-prepared','--domain',str(job['domain']),'--record-video'],
                    capture_output=True,text=True)
                (directory/'service_launch.log').write_text(result.stdout+result.stderr)
                if result.returncode:raise RuntimeError('Native pixel focus regression launch failed;inspect actual service metadata')
                job['status']='running';state['status']='running_pixel_focus_udp_regression';save(path,state)
            live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if job['status']=='running' and live.exists() and json.loads(live.read_text()).get('status',{}).get('state')=='running':
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
                        if check['mismatches']:raise RuntimeError('Native pixel focus policy parameters differ from expected source')
                        job['runtime_configuration_gate']=check['passed']
                if job.get('runtime_configuration_gate') and not job.get('velocity_profile_gate'):
                    report=observe(velocity_probe,job,'effective_velocity_profile.json')
                    if report is not None:
                        job['velocity_profile_gate']=velocity_gate(report)
                        if report.get('complete') and not job['velocity_profile_gate']:
                            raise RuntimeError('Native pixel focus motion profile differs from frozen calibrated source')
                save(path,state)
            time.sleep(3)
    finally:
        settle_claims(parent,dependency,output)
        resume_cem(parent)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','dependency','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    paths={name:getattr(args,name).resolve() for name in ('parent','dependency','workspace','output')}
    if args.prepare_only:prepare(paths['parent'],paths['dependency'],paths['workspace'],paths['output'])
    else:run(**paths)
