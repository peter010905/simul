"""Prepare new exterior-shape learning, preserve/drain the preceding runs."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from continual_native_curriculum import collect, digest, save, source_hashes
from native_tournament import service_active
from spatial_native_curriculum import plan

ROOT = Path(__file__).resolve().parents[1]
UNIT = 'osp-spatial-cem-v1.service'


def audit_geometry(root):
    sys.path.insert(0, str(ROOT/'src/osp_mission'))
    import yaml
    from osp_mission.space_geometry import rasterize_space, valid_space
    state = json.loads((root/'state.json').read_text())
    rows = []
    for job in state['jobs']:
        file = Path(job['scenario']['file'])
        if digest(file) != job['scenario']['sha256']:
            raise RuntimeError('Prepared scene changed')
        cfg = yaml.safe_load(file.read_text())
        grid = rasterize_space(cfg, .10)
        exterior = {k:cfg['arena'][k] for k in ('shape','polygon','walls')}
        rows.append(dict(number=job['number'], seed=cfg['seed'], role=job['role'],
            arena=job['scenario']['arena'], start=cfg['start'], actual_obstacles=len(cfg['obstacles']),
            reachable_approaches=valid_space(cfg),
            occupancy_sha256=hashlib.sha256(str(grid.shape).encode()+grid.tobytes()).hexdigest(),
            exterior_sha256=hashlib.sha256(json.dumps(exterior, sort_keys=True).encode()).hexdigest()))
    result = dict(evidence='Offline collision and exterior geometry; not a native success result',
        episodes=len(rows), shapes=sorted({r['arena']['shape'] for r in rows}),
        unique_exteriors=len({r['exterior_sha256'] for r in rows}),
        unique_occupancy=len({r['occupancy_sha256'] for r in rows}),
        all_approaches_reachable=all(r['reachable_approaches'] for r in rows), rows=rows)
    if result['unique_exteriors'] != len(rows) or result['unique_occupancy'] != len(rows) or not result['all_approaches_reachable']:
        raise RuntimeError('Exterior diversity/accessibility audit failed')
    save(root/'initial_geometry_audit.json', result)
    return result


def prepare_campaign(parent, destination):
    if (destination/'configuration.json').exists() or (destination/'state.json').exists():
        raise FileExistsError('Campaign already prepared; do not overwrite frozen experiments')
    config = copy.deepcopy(json.loads((parent/'configuration.json').read_text()))
    old = json.loads((parent/'state.json').read_text())
    candidate, baseline = destination/'runtime', destination/'baseline_runtime'
    for path in (candidate, baseline):
        if not (path/'install/setup.bash').is_file():
            raise RuntimeError(f'Build runtime before freezing: {path}')
    for file in ('candidate_tests_final.log','baseline_build_and_tests.log'):
        text = (destination/file).read_text()
        if '210 passed, 1 skipped' not in text or 'FAILED' in text:
            raise RuntimeError(f'Runtime checks must pass before launching: {file}')
    config.update(run_id='nov13-space', seed=1136000000, workspace=str(candidate),
        baseline_workspace=str(baseline), baseline_weights=old['champion'], parent_campaign=str(parent),
        purpose='Exterior shape and topology randomization with odometry-anchored exploration memory',
        training_episodes_per_generation=28,
        pilot_gate='7/7 paired shape pilots,0 contacts,>=20% lower PAR-2',
        validation_gate='>=26/28 paired native successes,>=3/4 per shape,0 contacts,>=20% lower PAR-2',
        spatial_final_gate='>=38/42 native successes,>=5/6 per shape,0 contacts',
        final_gate='After spatial gate:>=36/40 independent original PDF benchmarks,0 contacts',
        outer_shapes=['circle','triangle','trapezoid','rectangle','l_shape','u_shape','rooms'],
        circle_collision_model='48-segment closed polygon approximation',
        policy_receives_ground_truth=False)
    config['source_hashes'] = {str(p):source_hashes(p) for p in (candidate, baseline)}
    drivers = set(config['driver_hashes']) | {str(Path(__file__).resolve()), str(ROOT/'training_robust/spatial_native_curriculum.py')}
    drivers |= {str(ROOT/'src/osp_mission/osp_mission'/name) for name in ('space_geometry.py','space_scenarios.py','marker_sizes.py')}
    config['driver_hashes'] = {name:digest(Path(name)) for name in drivers}
    save(destination/'configuration.json', config)
    state = dict(status='running', generation=0, phase='train', planned=False, jobs=[],
        champion=old['champion'], mean=old['mean'], std=old['std'],
        reserved_holdout_consumed=old.get('reserved_holdout_consumed', False), spatial_holdout_consumed=False)
    (destination/'scenarios').mkdir(exist_ok=True)
    plan(destination, config, state)
    save(destination/'state.json', state)
    return audit_geometry(destination)


def start(parent, destination):
    subprocess.run(['systemctl','--user','stop','osp-fresh-map-cem-v2.service'], check=True)
    previous = json.loads((parent/'state.json').read_text())
    waiting = [j for j in previous['jobs'] if j['status'] == 'running']
    previous.update(status='superseded_by_spatial_cem_v1', successor=str(destination))
    for job in waiting:
        if not service_active(job['unit']):
            collect(job)
    save(parent/'state.json', previous)
    save(parent/'successor_transition.json', dict(status='draining_previous_native_jobs',
        destination=str(destination), units=[j['unit'] for j in waiting], original_results_preserved=True))
    subprocess.run(['systemd-run','--user',f'--unit={UNIT}',f'--property=WorkingDirectory={ROOT}',
        '--property=MemoryMax=512M','--property=Restart=no',
        f'--property=StandardOutput=append:{destination/"coordinator.log"}',
        f'--property=StandardError=append:{destination/"coordinator.log"}',
        '/usr/bin/python3','-u',str(ROOT/'training_robust/spatial_native_curriculum.py'),str(destination)], check=True)
    if not service_active(UNIT):
        raise RuntimeError('Spatial coordinator did not start')
    # This supervisor remains alive until the old native services terminate.
    # New dispatcher checks both their PIDs and ROS domains to avoid overlap.
    while any(service_active(j['unit']) for j in waiting):
        time.sleep(5)
    for job in waiting:
        collect(job)
    save(parent/'state.json', previous)
    save(parent/'learning_log.json', [dict(number=j['number'], phase=j['phase'],
        scenario=j['scenario'], parameters=j['parameters'], outcome=j['outcome'])
        for j in previous['jobs'] if j['status'] == 'done'])
    save(parent/'successor_transition.json', dict(status='successor_active', service=UNIT,
        destination=str(destination), original_results_preserved=True, goal_verified=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--start-prepared',action='store_true')
    args = parser.parse_args()
    parent, destination = args.parent.resolve(), args.destination.resolve()
    if not args.start_prepared:
        print(json.dumps({k:v for k,v in prepare_campaign(parent,destination).items() if k!='rows'}),flush=True)
    if not args.prepare_only:
        start(parent,destination)
