"""Insert one graph-update native regression while preserving fresh-map CEM."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time

from continual_native_curriculum import digest,save,source_hashes
from launch_spatial_cem_v6 import start_unit
from native_tournament import native_processes,service_active
from observe_spatial_campaign import native_pid
from probe_velocity_profile import probe as velocity_probe,gate as velocity_gate
from run_floor_pose_regression import probe as floor_probe,floor_gate
from run_windowed_witness_regression_v2 import settle_claims
from runtime_configuration_v2 import probe as config_probe,configuration_gate
from spatial_native_curriculum_v6 import benchmark_output_live,collect,observe

ROOT=Path(__file__).resolve().parents[1]
UNIT='osp-graph-landmark-regression-v1.service'


def prepare(parent,screen,workspace,output):
    reference=json.loads((screen/'state.json').read_text())['jobs'][2]
    if reference['status']!='done' or reference['outcome']['success']:
        raise ValueError('Require preserved finalized trapezoid coordinate failure')
    if '273 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require full candidate runtime tests and completed build')
    base=Path(json.loads((parent/'configuration.json').read_text())['workspace'])
    before,after=source_hashes(base),source_hashes(workspace)
    changed={p for p in before.keys()|after.keys() if before.get(p)!=after.get(p)}
    expected={'setup.py','osp_mission/mission.py','osp_mission/graph_landmarks.py',
        'test/test_graph_landmarks_runtime.py','tools/export_slam_graph_poses.cpp','tools/export_slam_graph_poses'}
    if changed!=expected:raise ValueError(f'Unexpected source changes:{changed}')
    output.mkdir(parents=True,exist_ok=False);case=output/'case_0';case.mkdir()
    shutil.copy2(Path(reference['output'])/'policy.json',case/'policy.json')
    manifest=json.loads((Path(reference['output'])/'manifest.json').read_text())
    manifest['policy_sha256']=digest(case/'policy.json');save(case/'manifest.json',manifest)
    state=dict(status='running',phase='diagnostic',training_samples=False,
        generalization_verified=False,production_promoted=False,parent_campaign=str(parent),
        coordinator_unit=UNIT,source_hashes=after,source_changes=sorted(changed),baseline=reference['outcome'],
        comparison_limit='S8 combines S7 windowed proof with final graph updates; reference S6 has original consecutive proof',
        driver_hashes={str(p):digest(p) for p in [Path(__file__),ROOT/'scripts/candidate_graph_landmarks.py',
            ROOT/'training_robust/export_slam_graph_poses.cpp',ROOT/'scripts/prepare_graph_landmarks_runtime.py']},
        jobs=[dict(number=0,generation=0,phase='diagnostic',role='candidate',workspace=str(workspace),
            output=str(case),scenario=reference['scenario'],parameters=reference['parameters'],
            status='prepared',domain=126,unit=UNIT.removesuffix('.service')+'-native.service')])
    save(output/'state.json',state)


def resume_cem(parent):
    state=json.loads((parent/'state.json').read_text())
    if state['status']!='running':raise RuntimeError('Do not override a terminal or repair-needed CEM state')
    unit=state['coordinator_unit']
    if service_active(unit):return
    load=subprocess.run(['systemctl','--user','show',unit,'-p','LoadState','--value'],
        capture_output=True,text=True,check=True).stdout.strip()
    if load=='loaded':
        result=subprocess.run(['systemctl','--user','start',unit],capture_output=True,text=True)
        if result.returncode==0 and service_active(unit):return
    # This campaign uses the v6 driver. The older resume helper hardcodes v5.
    start_unit(unit,['/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum_v6.py'),str(parent)],
        parent/'coordinator.log')
    if not service_active(unit):raise RuntimeError('Fresh-map CEM dispatcher did not resume')


def run(parent,legacy,screen,workspace,output):
    path=output/'state.json'
    if not path.exists():prepare(parent,screen,workspace,output)
    state=json.loads(path.read_text());job=state['jobs'][0]
    if job['status']=='done':return
    owner=json.loads((parent/'state.json').read_text())
    if owner['status']!='running':raise RuntimeError('Require an active CEM campaign')
    restored=False
    try:
        if service_active(owner['coordinator_unit']):
            subprocess.run(['systemctl','--user','stop',owner['coordinator_unit']],check=True)
        settle_claims(parent,legacy,output)
        while job['status']!='done':
            if source_hashes(workspace)!=state['source_hashes']:raise RuntimeError('Frozen graph runtime changed')
            if any(digest(Path(p))!=expected for p,expected in state['driver_hashes'].items()):
                raise RuntimeError('Frozen graph regression driver changed')
            directory=Path(job['output'])
            if job['status']=='prepared' and (directory/'service.json').exists():job['status']='running'
            pid=native_pid(job['unit'],directory) if job['status']=='running' else None
            if pid and not restored:
                resume_cem(parent);restored=True
                save(output/'dispatch_handoff.json',dict(verified_new_native_pid=pid,
                    resumed_unit=owner['coordinator_unit'],driver='spatial_native_curriculum_v6.py',
                    old_native_runs_and_prepared_scenes_preserved=True))
            if job['status']=='running' and not service_active(job['unit']) and not benchmark_output_live(directory):
                # Allow the read-only evaluator recorder to finalize its last buffered trace.
                time.sleep(2)
                collect(job)
                if not all(job.get(k) is True for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate')):
                    job['raw_outcome_before_input_gates']=dict(job['outcome'])
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='native_effective_inputs_unverified')
                result=json.loads((directory/'report.json').read_text())['rows'][0]['result'] if (directory/'report.json').exists() else {}
                update=result.get('graph_landmark_update',{})
                job['graph_update_gate']=bool(update.get('applied') and update.get('ground_truth_inputs') is False
                    and update.get('method')=='confirmed_odom_landmarks_reprojected_by_final_SLAM_scan_poses')
                if job['outcome']['success'] and not job['graph_update_gate']:
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='native_graph_update_not_observed')
                state['status']='completed_graph_landmark_regression';save(path,state);break
            if job['status']=='prepared' and len(native_processes())<2:
                result=subprocess.run(['/usr/bin/python3',str(ROOT/'training_robust/run_native_service.py'),
                    '--unit',job['unit'].removesuffix('.service'),'--workspace',str(workspace),'--output',str(directory),
                    '--','/usr/bin/python3',str(ROOT/'training_robust/gazebo_benchmark.py'),
                    '--output',str(directory),'--run-prepared','--domain',str(job['domain']),'--record-video'],
                    capture_output=True,text=True)
                (directory/'service_launch.log').write_text(result.stdout+result.stderr)
                if result.returncode:raise RuntimeError('Native graph regression launch failed;inspect actual service metadata')
                job['status']='running';state['status']='running_graph_landmark_regression';save(path,state)
            live=directory/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if job['status']=='running' and live.exists() and json.loads(live.read_text()).get('status',{}).get('state')=='running':
                if not job.get('floor_sensor_gate'):
                    report=observe(floor_probe,job,'floor_probe.json')
                    if report is not None and floor_gate(report):job['floor_sensor_gate']=True
                if job.get('floor_sensor_gate') and not job.get('runtime_configuration_gate'):
                    report=observe(config_probe,job,'runtime_configuration.json')
                    if report is not None:
                        check=configuration_gate(report,directory/'policy.json',1000)
                        save(directory/'runtime_configuration_check.json',check)
                        if check['mismatches']:raise RuntimeError('Native graph policy parameters differ from expected source')
                        job['runtime_configuration_gate']=check['passed']
                if job.get('runtime_configuration_gate') and not job.get('velocity_profile_gate'):
                    report=observe(velocity_probe,job,'effective_velocity_profile.json')
                    if report is not None:
                        job['velocity_profile_gate']=velocity_gate(report)
                        if report.get('complete') and not job['velocity_profile_gate']:
                            raise RuntimeError('Native graph motion profile differs from frozen calibrated source')
                save(path,state)
            time.sleep(3)
    finally:
        settle_claims(parent,legacy,output)
        resume_cem(parent)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','legacy','screen','workspace','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    paths={name:getattr(args,name).resolve() for name in ('parent','legacy','screen','workspace','output')}
    if args.prepare_only:prepare(paths['parent'],paths['screen'],paths['workspace'],paths['output'])
    else:run(**paths)
