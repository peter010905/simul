"""Safely hand two native slots to a frozen concave-arena curriculum.

Stops only the old dispatcher. Existing native runs, old queues and frozen
runtime sources remain untouched; the new runner counts their occupied slots.
"""
import argparse
import copy
import datetime
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from continual_native_curriculum import digest,save,source_hashes
from launch_spatial_cem_v6 import start_unit
from launch_spatial_curriculum import audit_geometry
from native_tournament import service_active
from run_directed_integrated_screen_v2 import prime_parent_inputs
from run_windowed_witness_regression_v2 import settle_claims
import spatial_native_curriculum_v6 as predecessor
import challenge_native_curriculum as successor

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/'training_robust'
UNIT='osp-concave-challenge-v1.service'


def prepare(parent,regression,destination):
    trial=json.loads((regression/'state.json').read_text());candidate=trial['jobs'][0]
    if trial['status']!='completed_face_intent_regression' or not candidate['outcome']['success']:
        raise ValueError('Require completed independently verified native face-orientation pass')
    gates=('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate','slam_transport_gate',
           'physical_credit_gate','witness_algorithm_gate','face_directed_profile_gate')
    if not all(candidate.get(k) is True for k in gates):raise ValueError('Missing native input/physical evidence gates')
    workspace=Path(candidate['workspace']);old=json.loads((parent/'configuration.json').read_text())
    baseline=Path(old['workspace'])
    if source_hashes(workspace)!=trial['source_hashes']:raise ValueError('Native-tested candidate changed')
    destination.mkdir(parents=True,exist_ok=False);(destination/'scenarios').mkdir()
    config=copy.deepcopy(old)
    config.update(run_id='nov13-concave1',seed=1192700000,workspace=str(workspace),baseline_workspace=str(baseline),
        baseline_weights=candidate['parameters'],runtime_alternatives=[str(workspace)],domains=[141,142],
        parent_campaign=str(parent),native_engineering_reference=str(regression),
        purpose='C/ㄷ, U and zigzag hidden-obstacle search; fresh geometry for every training mission',
        primary_baseline='Frozen windowed-witness predecessor at identical initial weights',
        outer_shapes=['c_shape','u_shape','zigzag'],mount_profiles=['hidden_obstacles'],
        training_episodes_per_generation=12,pilot_gate='3/3candidate;0contacts;>=20%lowerPAR2 paired',
        validation_gate='>=11/12candidate;>=3/4per shape;0contacts;>=20%lowerPAR2 paired',
        spatial_final_gate='>=17/18candidate;>=5/6per shape;0contacts',
        final_gate='After challenge gate:>=36/40independent originalPDF;0contacts',
        generalization_verified=False,production_promoted=False,policy_receives_ground_truth=False)
    config['source_hashes']={str(p):source_hashes(p) for p in (workspace,baseline)}
    # Existing frozen drivers remain included. No existing driver is edited.
    files=set(old['driver_hashes'])|{str(Path(__file__).resolve()),str(HERE/'challenge_native_curriculum.py'),
        str(ROOT/'src/osp_mission/osp_mission/challenge_scenarios.py'),
        str(HERE/'probe_encoder_delta_profile.py'),str(HERE/'run_encoder_delta_regression.py')}
    config['driver_hashes']={p:digest(Path(p)) for p in files}
    save(destination/'configuration.json',config)
    parent_state=json.loads((parent/'state.json').read_text());weights=candidate['parameters']
    state=dict(status='running',generation=0,phase='train',planned=False,jobs=[],champion=weights,
        champion_workspace=str(workspace),mean=np.log(weights).tolist(),std=[0.,.4,.4,.4,.4],
        reserved_holdout_consumed=parent_state.get('reserved_holdout_consumed',False),spatial_holdout_consumed=False,
        coordinator_unit=UNIT,source_fixed_during_campaign=True)
    successor.plan(destination,config,state)
    used={j.get('scenario',{}).get('seed') for p in parent.parent.glob('*/state.json') if p.parent!=destination
          for j in json.loads(p.read_text()).get('jobs',[])}
    if any(j['scenario']['seed'] in used for j in state['jobs']):raise ValueError('Previously prepared training seed')
    save(destination/'state.json',state)
    return audit_geometry(destination)


def drain(parent,destination):
    path=parent/'state.json'
    while True:
        state=json.loads(path.read_text());waiting=[]
        if service_active(state['coordinator_unit']):raise RuntimeError('Old dispatcher unexpectedly resumed')
        for job in state['jobs']:
            if job['status']=='prepared' and (Path(job['output'])/'service.json').exists():job['status']='running'
            if job['status']!='running':continue
            if service_active(job['unit']) or predecessor.benchmark_output_live(job['output']):
                waiting.append(job['unit']);continue
            predecessor.collect(job)
            if not all(job.get(k) for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate')):
                job['raw_outcome_before_transition_input_gate']=dict(job['outcome'])
                job['outcome'].update(success=False,infrastructure_failure=True,reason='legacy_native_inputs_unverified')
        if not waiting:state['status']='superseded_by_concave_challenge_v1'
        save(path,state)
        save(destination/'legacy_drain.json',dict(waiting_native_units=waiting,old_native_runs_preserved=True,
                                                old_queued_artifacts_retained=True))
        if not waiting:return
        time.sleep(5)


def launch(parent,regression,destination):
    if not (destination/'state.json').exists():prepare(parent,regression,destination)
    trial=json.loads((regression/'state.json').read_text())
    if service_active(trial['coordinator_unit']):raise RuntimeError('Prior diagnostic coordinator still active')
    if (destination/'transition_started.json').exists():
        if not service_active(UNIT):raise RuntimeError('Inspect original transition; do not start a duplicate coordinator')
        return drain(parent,destination)
    held=parent.parent/'directed_integrated_screen_v1';screen=json.loads((held/'state.json').read_text())
    if screen['status']!='held_for_face_source_training_upgrade' or service_active(screen['coordinator_unit']):
        raise RuntimeError('Require preserved held engineering queue')
    old=json.loads((parent/'state.json').read_text())
    if service_active(old['coordinator_unit']):subprocess.run(['systemctl','--user','stop',old['coordinator_unit']],check=True)
    settle_claims(parent,held,destination);prime_parent_inputs(parent)
    old=json.loads((parent/'state.json').read_text());save(parent/'pre_challenge_transition_state.json',old)
    old.update(status='draining_to_concave_challenge_v1',successor=str(destination),original_prepared_jobs_retained=True)
    save(parent/'state.json',old)
    save(destination/'transition_started.json',dict(parent=str(parent),native_runs_not_stopped=True,
        maximum_native_concurrency=2,created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
    start_unit(UNIT,['/usr/bin/python3','-u',str(HERE/'challenge_native_curriculum.py'),str(destination)],destination/'coordinator.log')
    for unit,script,memory in [('osp-challenge-evaluator-v1','observe_spatial_campaign.py','512M'),
            ('osp-challenge-failures-v1','capture_native_failures.py','256M')]:
        start_unit(unit+'.service',['/usr/bin/python3','-u',str(HERE/script),str(destination)],destination/(unit+'.log'),memory)
    drain(parent,destination)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','regression','destination'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    paths=[getattr(a,n).resolve() for n in ('parent','regression','destination')]
    if a.prepare_only:print(json.dumps(prepare(*paths)))
    else:launch(*paths)
