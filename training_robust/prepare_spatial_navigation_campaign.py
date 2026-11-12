"""Prepare fresh exterior learning only after an audited native diagnostic win."""
import argparse
import copy
import json
from pathlib import Path
import subprocess

from continual_native_curriculum import digest,save,source_hashes
from launch_spatial_curriculum import audit_geometry
from native_tournament import service_active
from spatial_native_curriculum_v4 import plan

ROOT=Path(__file__).resolve().parents[1]


def prepare(parent,regression,destination):
    previous=json.loads((parent/'state.json').read_text())
    diagnostic=json.loads((regression/'state.json').read_text())
    if diagnostic['status']!='completed_navigation_regression':
        raise RuntimeError('Wait for finalized native videos and independent approach/position audits')
    if any(j['status']=='running' or service_active(j['unit']) for j in previous['jobs']):
        raise RuntimeError('Preserve and drain previous native jobs before superseding')
    choices=[j for j in diagnostic['jobs'] if j['outcome']['success']
        and j.get('floor_sensor_gate') and j.get('runtime_configuration_gate')]
    if not choices:raise RuntimeError('No successful diagnostic candidate; retain the previous campaign')
    # Prefer persistent physical approach proof when it also passes all actual
    # rubric checks. This chooses a learning candidate, never a release policy.
    choices.sort(key=lambda j:(j['number']==1,-j['outcome']['seconds']),reverse=True)
    winner=choices[0];candidate=Path(winner['workspace'])
    reference=Path(json.loads((parent/'configuration.json').read_text())['workspace'])
    for workspace in (candidate,reference):
        if not (workspace/'install/setup.bash').exists():raise RuntimeError('Build candidate first')
    if source_hashes(candidate)!=diagnostic['source_hashes'][str(candidate)]:
        raise RuntimeError('Diagnostic candidate changed')
    destination.mkdir(parents=True,exist_ok=False);(destination/'scenarios').mkdir()
    config=copy.deepcopy(json.loads((parent/'configuration.json').read_text()))
    config.update(run_id='nov13-space4',seed=1161000000,workspace=str(candidate),
        baseline_workspace=str(reference),baseline_weights=previous['champion'],
        baseline_action_acknowledgement_timeout_ms=1000,baseline_live_local_costmap=False,
        parent_campaign=str(parent),navigation_diagnostic=str(regression),
        purpose='Fresh exterior CEM with sensor-local collision maps and independent actual-approach audit',
        runtime_configuration_gate_required=True,
        action_acknowledgement_effect='1000ms used in both arms; not attributed to this comparison',
        local_costmap_change='Rolling odom map uses live voxel + inflation; global SLAM/static map retained',
        approach_witness_enabled=winner['number']==1,
        generalization_verified=False,production_promoted=False)
    drivers=[ROOT/'training_robust'/name for name in (
        'spatial_native_curriculum_v4.py','continual_native_curriculum.py','audit_native_campaign.py',
        'native_tournament.py','run_native_service.py','gazebo_benchmark.py','run_floor_pose_regression.py',
        'runtime_configuration_v2.py')]
    drivers+=[ROOT/'scripts/probe_floor_sensors.py']
    drivers+=[ROOT/'src/osp_mission/osp_mission'/name for name in (
        'space_geometry.py','space_scenarios.py','marker_sizes.py','robust_scenarios.py','scenarios.py')]
    config['driver_hashes']={str(p):digest(p) for p in drivers}
    config['source_hashes']={str(p):source_hashes(p) for p in (candidate,reference)}
    save(destination/'configuration.json',config)
    state=dict(status='running',generation=0,phase='train',planned=False,jobs=[],
        champion=previous['champion'],mean=previous['mean'],std=previous['std'],
        reserved_holdout_consumed=previous.get('reserved_holdout_consumed',False),
        spatial_holdout_consumed=False,coordinator_unit='osp-spatial-cem-v4.service')
    plan(destination,config,state);save(destination/'state.json',state)
    audit=audit_geometry(destination)
    prior_audit=json.loads((parent/'initial_geometry_audit.json').read_text())
    if {r['exterior_sha256'] for r in audit['rows']} & {r['exterior_sha256'] for r in prior_audit['rows']}:
        raise RuntimeError('Fresh learning must not reuse previous exterior shapes')
    save(destination/'preparation.json',dict(diagnostic_winner=winner['number'],
        diagnostic_outcome=winner['outcome'],diagnostic_is_training_sample=False,
        generalization_verified=False,production_promoted=False,
        candidate_package_sha256=config['source_hashes'][str(candidate)],
        new_exteriors=True,independent_actual_approach_radius_m=1.5))
    return audit


def launch(parent,destination):
    state=json.loads((parent/'state.json').read_text())
    if state['status']!='held_for_navigation_regression':raise RuntimeError('Predecessor dispatcher state changed')
    subprocess.run(['systemd-run','--user','--unit=osp-spatial-cem-v4',
        f'--property=WorkingDirectory={ROOT}','--property=MemoryMax=512M','--property=Restart=no',
        f'--property=StandardOutput=append:{destination/"coordinator.log"}',
        f'--property=StandardError=append:{destination/"coordinator.log"}',
        '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum_v4.py'),
        str(destination)],check=True)
    state.update(status='superseded_by_spatial_cem_v4',successor=str(destination),
        original_native_results_preserved=True)
    save(parent/'state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('parent','regression','destination'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--launch',action='store_true')
    args=parser.parse_args();parent=args.parent.resolve();regression=args.regression.resolve();dest=args.destination.resolve()
    report=prepare(parent,regression,dest)
    print(json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
    if args.launch:launch(parent,dest)
