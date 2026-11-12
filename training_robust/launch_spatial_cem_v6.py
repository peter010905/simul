"""Move training to the corrected source; drain all legacy native runs intact."""
import argparse
import copy
import datetime
import json
from pathlib import Path
import subprocess
import time

import numpy as np

from audit_approach_evidence import audit as audit_credits
from continual_native_curriculum import digest, save, source_hashes
from launch_spatial_curriculum import audit_geometry
from native_tournament import service_active
from observe_spatial_campaign import native_pid
from probe_velocity_profile import gate as velocity_gate
from run_windowed_witness_regression_v2 import settle_claims
from spatial_native_curriculum_v5 import collect as collect_legacy
from spatial_native_curriculum_v6 import plan, strict_credit_gate, benchmark_output_live, observe

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/'training_robust'
UNIT = 'osp-spatial-cem-v6.service'


def prepare(parent, screen, regression, destination):
    trial = json.loads((regression/'state.json').read_text()); candidate = trial['jobs'][0]
    reference = json.loads((screen/'state.json').read_text())['jobs'][0]
    observation = json.loads((Path(candidate['output'])/'observer_analysis.json').read_text())
    if not (candidate['status'] == 'done' and candidate['outcome']['success']
            and trial.get('velocity_profile_verified') and strict_credit_gate(observation['physical_credit_audit'])
            and candidate['outcome']['seconds'] <= .8*reference['outcome']['seconds']):
        raise ValueError('Require full native same-scene engineering pass and 20% timing improvement before training')
    workspace, baseline = Path(candidate['workspace']), Path(reference['workspace'])
    if '259 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require the corrected runtime full test suite')
    destination.mkdir(parents=True, exist_ok=False); (destination/'scenarios').mkdir()
    config = copy.deepcopy(json.loads((parent/'configuration.json').read_text()))
    weights = candidate['parameters']
    config.update(run_id='nov13-space6', seed=1191300000, workspace=str(workspace),
        baseline_workspace=str(baseline), baseline_weights=reference['parameters'],
        runtime_alternatives=[str(workspace)], domains=[124,125], parent_campaign=str(parent),
        prior_engineering_screen=str(screen), native_engineering_reference=str(regression),
        baseline_action_acknowledgement_timeout_ms=1000, baseline_live_local_costmap=True,
        purpose='Fresh-space coefficient learning on corrected sensor/SLAM/camera/near-proof source',
        primary_baseline='Frozen calibrated predecessor with identical camera/control/SLAM and original consecutive witness',
        training_episodes_per_generation=28,
        pilot_gate='7/7candidate paired shapes,0contacts,>=20%lowerPAR2 against calibrated predecessor',
        validation_gate='>=26/28candidate,>=3/4perfamily,0contacts,>=20%lowerPAR2',
        spatial_final_gate='>=38/42candidate,>=5/6perfamily,0contacts',
        final_gate='After spatial gate:>=36/40independent originalPDF,0contacts',
        exact_physical_credit_gate=True, policy_receives_ground_truth=False)
    config['source_hashes'] = {str(p):source_hashes(p) for p in (workspace,baseline)}
    drivers = set(config['driver_hashes']) | {str(Path(__file__).resolve()),
        str(HERE/'spatial_native_curriculum_v6.py'), str(HERE/'audit_approach_evidence.py'),
        str(HERE/'probe_velocity_profile.py'), str(HERE/'run_windowed_witness_regression_v2.py')}
    config['driver_hashes'] = {p:digest(Path(p)) for p in drivers}
    save(destination/'configuration.json',config)
    state = dict(status='running',generation=0,phase='train',planned=False,jobs=[],
        champion=weights,champion_workspace=str(workspace),mean=np.log(weights).tolist(),std=[0.,.4,.4,.4,.4],
        reserved_holdout_consumed=False,spatial_holdout_consumed=False,
        coordinator_unit=UNIT,source_fixed_during_campaign=True)
    plan(destination,config,state)
    used = {j.get('scenario',{}).get('seed') for p in parent.parent.glob('*/state.json')
        if p.parent != destination for j in json.loads(p.read_text()).get('jobs',[])}
    if any(j['scenario']['seed'] in used for j in state['jobs']):
        raise ValueError('Do not reuse any previous prepared training seed')
    save(destination/'state.json',state)
    return audit_geometry(destination)


def start_unit(unit, command, log, memory='512M'):
    subprocess.run(['systemd-run','--user','--unit='+unit.removesuffix('.service'),
        f'--property=WorkingDirectory={ROOT}',f'--property=MemoryMax={memory}','--property=Restart=no',
        f'--property=StandardOutput=append:{log}',f'--property=StandardError=append:{log}',*command],
        capture_output=True,text=True,check=True)


def legacy_profile(job, kind, config):
    if job.get('floor_sensor_gate') is True and job.get('runtime_configuration_gate') is True: return
    episode = Path(job['output'])/'episodes'/str(job['scenario']['seed'])
    live = episode/'live_status.json'
    if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state') != 'running': return
    from run_floor_pose_regression import probe as floor_probe, floor_gate
    from runtime_configuration_v2 import probe as config_probe, configuration_gate
    if job.get('floor_sensor_gate') is not True:
        measured = observe(floor_probe,job,'floor_probe.json')
        if measured is not None: job['floor_sensor_gate'] = floor_gate(measured)
    if job.get('runtime_configuration_gate') is not True:
        report = observe(config_probe,job,'runtime_configuration.json')
        if report is None: return
        baseline = kind == 'cem' and job.get('role') == 'baseline'
        ack = config['baseline_action_acknowledgement_timeout_ms'] if baseline else 1000
        live_local = config['baseline_live_local_costmap'] if baseline else True
        check = configuration_gate(report,Path(job['output'])/'policy.json',ack,live_local=live_local)
        job['runtime_configuration_gate'] = check['passed']
        save(Path(job['output'])/'runtime_configuration_check.json',check)


def drain(roots, destination):
    while True:
        waiting = []
        for root,kind in roots:
            path = root/'state.json'; state = json.loads(path.read_text())
            config = json.loads((root/'configuration.json').read_text()) if kind == 'cem' else {}
            for job in state['jobs']:
                if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():
                    job['status']='running'
                if job['status'] != 'running': continue
                if service_active(job['unit']) or benchmark_output_live(job['output']):
                    legacy_profile(job,kind,config); waiting.append(job['unit']); continue
                collect_legacy(job)
                if not (job.get('floor_sensor_gate') and job.get('runtime_configuration_gate')):
                    job['raw_outcome_before_transition_input_gate'] = dict(job['outcome'])
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='legacy_native_inputs_unverified')
                if kind == 'screen':
                    output = Path(job['output']); episode = output/'episodes'/str(job['scenario']['seed'])
                    profile = output.parent/'effective_velocity_profile.json'
                    job['velocity_profile_verified'] = profile.exists() and velocity_gate(json.loads(profile.read_text()))
                    try:
                        audit = audit_credits(episode,Path(job['scenario']['file']))
                        save(output/'physical_credit_audit.json',audit)
                        physical = strict_credit_gate(audit)
                    except (OSError,ValueError,KeyError,TypeError): physical=False
                    job['physical_credit_verified'] = physical
                    job['all_range_witnesses_verified'] = physical
                    job['strict_screen_passed'] = bool(job['outcome']['success'] and physical and job['velocity_profile_verified'])
            if not any(j['status']=='running' for j in state['jobs']):
                state['status']='superseded_by_spatial_cem_v6'
            save(path,state)
        save(destination/'legacy_drain.json',dict(waiting_native_units=waiting,
            old_native_runs_preserved=True,old_queued_artifacts_retained=True))
        if not waiting: return
        time.sleep(5)


def launch(parent,screen,regression,destination):
    if not (destination/'state.json').exists(): prepare(parent,screen,regression,destination)
    if (destination/'transition_started.json').exists():
        if json.loads((destination/'state.json').read_text())['status']=='running' and not service_active(UNIT):
            raise RuntimeError('Transition recorded but successor coordinator is not active;inspect its actual unit before retrying')
        return drain([(parent,'cem'),(screen,'screen')],destination)
    roots = [(parent,'cem'),(screen,'screen')]
    for root,kind in roots:
        state = json.loads((root/'state.json').read_text())
        if service_active(state['coordinator_unit']):
            subprocess.run(['systemctl','--user','stop',state['coordinator_unit']],check=True)
    settle_claims(parent,screen)
    for root,kind in roots:
        path=root/'state.json';state=json.loads(path.read_text());save(root/'pre_v6_transition_state.json',state)
        if kind == 'screen':
            for index,job in enumerate(state['jobs']):
                child=root/f'screen_{index}/state.json'
                if job['status']!='done' and child.exists():
                    current=json.loads(child.read_text())['jobs'][0]
                    job.update(current);job['number']=index
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():job['status']='running'
        state.update(status='draining_to_spatial_cem_v6',successor=str(destination),
            original_prepared_jobs_retained=True,transition_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        save(path,state)
    save(destination/'transition_started.json',dict(legacy_roots=[str(root) for root,_ in roots],
        native_runs_not_stopped=True,old_sources_and_queues_retained=True))
    start_unit(UNIT,['/usr/bin/python3','-u',str(HERE/'spatial_native_curriculum_v6.py'),str(destination)],destination/'coordinator.log')
    start_unit('osp-spatial-evaluator-observer-v6.service',['/usr/bin/python3','-u',str(HERE/'observe_spatial_campaign.py'),str(destination)],destination/'evaluator_observer.log')
    start_unit('osp-native-failure-evidence-v6.service',['/usr/bin/python3','-u',str(HERE/'capture_native_failures.py'),str(destination)],destination/'failure_evidence.log','256M')
    drain(roots,destination)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','screen','regression','destination'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
    paths=[getattr(args,name).resolve() for name in ('parent','screen','regression','destination')]
    if args.prepare_only:print(json.dumps(prepare(*paths),indent=2))
    else:launch(*paths)
