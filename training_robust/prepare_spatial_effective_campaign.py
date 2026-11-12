"""Supersede a candidate whose Gazebo profile did not apply the intended timeout."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import time

from continual_native_curriculum import collect,digest,save,source_hashes
from launch_spatial_curriculum import audit_geometry
from native_tournament import service_active
from spatial_native_curriculum_v3 import plan

ROOT=Path(__file__).resolve().parents[1]


def prepare(parent,calibration,candidate,reference,destination):
    before=json.loads((parent/'configuration.json').read_text())
    prior=json.loads((parent/'state.json').read_text())
    sensor=json.loads((calibration/'state.json').read_text())
    if not all(j.get('floor_sensor_gate') is True for j in sensor['jobs']) or len(sensor['jobs'])!=2:
        raise RuntimeError('Require two independently measured native floor probes')
    floor_source=Path(sensor['jobs'][0]['workspace'])
    actual=source_hashes(candidate);calibrated=source_hashes(floor_source)
    differences={name for name in actual.keys()|calibrated.keys() if actual.get(name)!=calibrated.get(name)}
    if differences!={'config/nav2_gazebo.yaml','config/nav2_robot_policy.yaml'}:
        raise RuntimeError(f'Unexpected changes from calibrated runtime:{differences}')
    if '217 passed, 1 skipped' not in (candidate.parent/'build_and_tests.log').read_text():
        raise RuntimeError('Candidate build/runtime checks have not passed')
    destination.mkdir(parents=True,exist_ok=False)
    (destination/'scenarios').mkdir()
    config=copy.deepcopy(before)
    config.update(run_id='nov13-space3',seed=1159000000,workspace=str(candidate),
        baseline_workspace=str(reference),parent_campaign=str(parent),
        purpose='Fresh exterior-shape CEM with calibrated floor and per-run effective ROS configuration audit',
        runtime_configuration_gate_required=True,action_acknowledgement_timeout_ms=1000,
        action_acknowledgement_effect='Actual Gazebo profile patched; speed effect remains for paired native validation')
    drivers=[ROOT/'training_robust'/name for name in (
        'spatial_native_curriculum_v3.py','continual_native_curriculum.py','audit_native_campaign.py',
        'native_tournament.py','run_native_service.py','gazebo_benchmark.py','run_floor_pose_regression.py',
        'runtime_configuration.py')]
    drivers += [ROOT/'scripts/probe_floor_sensors.py']
    drivers += [ROOT/'src/osp_mission/osp_mission'/name for name in (
        'space_geometry.py','space_scenarios.py','marker_sizes.py','robust_scenarios.py','scenarios.py')]
    config['driver_hashes']={str(p):digest(p) for p in drivers}
    config['source_hashes']={str(p):source_hashes(p) for p in (candidate,reference)}
    save(destination/'configuration.json',config)
    state=dict(status='running',generation=0,phase='train',planned=False,jobs=[],
        champion=prior['champion'],mean=prior['mean'],std=prior['std'],
        reserved_holdout_consumed=prior.get('reserved_holdout_consumed',False),
        spatial_holdout_consumed=False,coordinator_unit='osp-spatial-cem-v3.service')
    plan(destination,config,state);save(destination/'state.json',state)
    audit=audit_geometry(destination)
    previous=json.loads((parent/'initial_geometry_audit.json').read_text())
    if {r['exterior_sha256'] for r in audit['rows']} & {r['exterior_sha256'] for r in previous['rows']}:
        raise RuntimeError('New learning must not reuse preceding exteriors')
    save(destination/'preparation.json',dict(driver_sha256=digest(Path(__file__).resolve()),
        previous_actual_gazebo_ack_timeout_ms=20,previous_intended_ack_timeout_ms=1000,
        source_changes_from_calibrated_floor=sorted(differences),
        current_runtime_effects_verified=False,new_exteriors=True))
    return audit


def launch_and_drain(parent,destination):
    previous=json.loads((parent/'state.json').read_text())
    previous.update(status='superseded_by_spatial_cem_v3',successor=str(destination),
        actual_gazebo_ack_timeout_ms=20,intended_ack_timeout_applied=False)
    save(parent/'state.json',previous)
    subprocess.run(['systemd-run','--user','--unit=osp-spatial-cem-v3',
        f'--property=WorkingDirectory={ROOT}','--property=MemoryMax=512M','--property=Restart=no',
        f'--property=StandardOutput=append:{destination/"coordinator.log"}',
        f'--property=StandardError=append:{destination/"coordinator.log"}',
        '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum_v3.py'),str(destination)],check=True)
    waiting=[j for j in previous['jobs'] if j['status']=='running']
    save(parent/'successor_transition.json',dict(status='new_coordinator_live;preserving_current_native_runs',
        successor=str(destination),waiting_units=[j['unit'] for j in waiting]))
    while any(service_active(j['unit']) for j in waiting):
        time.sleep(5)
    for job in waiting:collect(job)
    save(parent/'state.json',previous)
    save(parent/'learning_log.json',[dict(number=j['number'],scenario=j['scenario'],
        parameters=j['parameters'],outcome=j['outcome'],actual_gazebo_ack_timeout_ms=20)
        for j in previous['jobs'] if j['status']=='done'])
    save(parent/'successor_transition.json',dict(status='successor_active',successor=str(destination),
                                              original_results_preserved=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','calibration','candidate','reference','destination'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--launch',action='store_true')
    args=parser.parse_args()
    values=[getattr(args,name).resolve() for name in ('parent','calibration','candidate','reference','destination')]
    report=prepare(*values)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
    if args.launch:launch_and_drain(args.parent.resolve(),args.destination.resolve())
