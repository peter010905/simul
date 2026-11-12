"""Prepare fresh shape learning after native floor sensors pass calibration."""
import argparse
import copy
import json
from pathlib import Path
import subprocess

from continual_native_curriculum import digest, save, source_hashes
from launch_spatial_curriculum import audit_geometry
from spatial_native_curriculum_v2 import plan

ROOT=Path(__file__).resolve().parents[1]


def prepare(parent,regression,candidate,reference,destination):
    calibration=json.loads((regression/'state.json').read_text())
    if len(calibration['jobs'])!=2 or not all(j.get('floor_sensor_gate') is True for j in calibration['jobs']):
        raise RuntimeError('Both actual native floor sensor probes must pass first')
    if source_hashes(Path(calibration['jobs'][0]['workspace']))!=calibration['candidate_source_hashes']:
        raise RuntimeError('Calibrated floor runtime source changed')
    for file in (candidate.parent/'build_and_tests.log',reference.parent/'baseline_build_and_tests.log'):
        text=file.read_text()
        if '217 passed, 1 skipped' not in text or 'FAILED' in text:
            raise RuntimeError(f'Runtime checks incomplete: {file}')
    old=json.loads((parent/'state.json').read_text())
    if old['status']!='invalidated_visual_floor_geometry':
        raise RuntimeError('Preserve and invalidate faulty floor runs before preparing new training')
    destination.mkdir(parents=True,exist_ok=False)
    (destination/'scenarios').mkdir()
    config=copy.deepcopy(json.loads((parent/'configuration.json').read_text()))
    config.update(run_id='nov13-space2',seed=1148000000,workspace=str(candidate),
        baseline_workspace=str(reference),parent_campaign=str(parent),
        purpose='Fresh exterior-shape CEM with corrected floor pose, per-run cliff validation and odometry revisit memory',
        floor_calibration=str(regression),floor_sensor_gate_required=True,
        action_acknowledgement_timeout_ms=1000,
        action_acknowledgement_effect='Candidate for broad paired native validation; not independently proven faster',
        baseline_weights=old['champion'])
    drivers=[ROOT/'training_robust'/name for name in (
        'spatial_native_curriculum_v2.py','continual_native_curriculum.py','audit_native_campaign.py',
        'native_tournament.py','run_native_service.py','gazebo_benchmark.py','run_floor_pose_regression.py')]
    drivers += [ROOT/'scripts/probe_floor_sensors.py']
    drivers += [ROOT/'src/osp_mission/osp_mission'/name for name in (
        'space_geometry.py','space_scenarios.py','marker_sizes.py','robust_scenarios.py','scenarios.py')]
    config['driver_hashes']={str(p):digest(p) for p in drivers}
    config['source_hashes']={str(p):source_hashes(p) for p in (candidate,reference)}
    save(destination/'configuration.json',config)
    state=dict(status='running',generation=0,phase='train',planned=False,jobs=[],
        champion=old['champion'],mean=old['mean'],std=old['std'],
        reserved_holdout_consumed=old.get('reserved_holdout_consumed',False),spatial_holdout_consumed=False,
        coordinator_unit='osp-spatial-cem-v2.service')
    plan(destination,config,state)
    save(destination/'state.json',state)
    audit=audit_geometry(destination)
    previous=json.loads((parent/'initial_geometry_audit.json').read_text())
    if {r['exterior_sha256'] for r in audit['rows']} & {r['exterior_sha256'] for r in previous['rows']}:
        raise RuntimeError('Fresh training must not reuse the preceding exterior geometries')
    save(destination/'preparation.json',dict(driver_sha256=digest(Path(__file__).resolve()),
        invalid_floor_outcomes_used_for_learning=False,calibration_source_hashes=calibration['candidate_source_hashes'],
        calibration_finished=calibration['status']=='completed_calibration_requires_fresh_training',
        fresh_exteriors_not_in_parent=True))
    return audit


def launch(destination):
    subprocess.run(['systemd-run','--user','--unit=osp-spatial-cem-v2',
        f'--property=WorkingDirectory={ROOT}','--property=MemoryMax=512M','--property=Restart=no',
        f'--property=StandardOutput=append:{destination/"coordinator.log"}',
        f'--property=StandardError=append:{destination/"coordinator.log"}',
        '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum_v2.py'),str(destination)],check=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','regression','candidate','reference','destination'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--launch',action='store_true')
    args=parser.parse_args()
    values=[getattr(args,name).resolve() for name in ('parent','regression','candidate','reference','destination')]
    audit=prepare(*values)
    print(json.dumps({k:v for k,v in audit.items() if k!='rows'}),flush=True)
    if args.launch:
        launch(args.destination.resolve())
