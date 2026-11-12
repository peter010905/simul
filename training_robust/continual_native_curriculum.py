"""Fresh-map CEM search with native Gazebo evaluation and paired time checks.

Every training job gets a unique physical layout. Only validation pairs share a
layout, to measure improvement against the frozen previous champion fairly.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import yaml

from audit_native_campaign import audit_episode
from native_tournament import native_processes, service_active

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'training_robust'


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes(workspace):
    package = Path(workspace) / 'src/osp_mission'
    return {str(p.relative_to(package)): digest(p) for p in package.rglob('*')
            if p.is_file() and not {'__pycache__', '.pytest_cache'}.intersection(p.parts)}


def quality(outcomes):
    """PAR-2 assigns 1200s to every failure, including infrastructure failures."""
    count = len(outcomes)
    successes = sum(bool(row['success']) for row in outcomes)
    penalized = sum(row['seconds'] if row['success'] else 1200. for row in outcomes)
    return dict(episodes=count, successes=successes, success_rate=successes / count,
                contacts=sum(row.get('contacts', 0) for row in outcomes),
                par2_seconds=penalized / count,
                mean_confirmed=sum(row.get('targets', 0) for row in outcomes) / count)


def rank_key(row):
    return (row['success_rate'], -row['contacts'], -row['par2_seconds'], row['mean_confirmed'])


def materially_better(candidate, baseline, minimum_episodes, minimum_success=.9):
    return (candidate['episodes'] >= minimum_episodes
            and candidate['success_rate'] >= minimum_success
            and candidate['success_rate'] >= baseline['success_rate']
            and candidate['contacts'] == 0
            and candidate['par2_seconds'] <= .8 * baseline['par2_seconds'])


def population(state, seed):
    rng = np.random.default_rng(seed + state['generation'])
    weights = np.exp(rng.normal(state['mean'], state['std'], size=(4, 5)).clip(-3, 3))
    weights[:, 0] = 1.
    weights[0] = state['champion']
    return weights.tolist()


def fresh_scene(seed, difficulty, profile):
    # These data are consumed by the simulator/evaluator, never by the policy.
    sys.path.insert(0, str(ROOT / 'src/osp_mission'))
    from osp_mission.marker_sizes import randomize_sizes
    from osp_mission.robust_scenarios import make_robust
    from osp_mission.scenarios import valid, reachable, rasterize
    from scipy.ndimage import distance_transform_edt
    for attempt in range(30):
        actual_seed = seed + attempt
        try:
            cfg = randomize_sizes(make_robust(actual_seed, difficulty, profile),
                                  actual_seed + 20000000, .08, .30, 'mixed')
        except (ValueError, RuntimeError):
            continue
        resolution = .15
        allowed = reachable(cfg, resolution) & (distance_transform_edt(~rasterize(cfg, resolution))*resolution >= .45)
        ys, xs = np.where(allowed)
        if profile == 'hidden_obstacles':
            unseen_side = np.ones(len(xs), dtype=bool)
            for target in cfg['targets']:
                unseen_side &= (((xs+.5)*resolution-target['x'])*np.cos(target['yaw'])
                                + ((ys+.5)*resolution-target['y'])*np.sin(target['yaw'])) < -.25
            ys, xs = ys[unseen_side], xs[unseen_side]
        if not len(xs):
            continue
        rng = np.random.default_rng(actual_seed + 30000000)
        start_index = int(rng.integers(len(xs)))
        cfg['start'] = [float((xs[start_index]+.5)*resolution),
                        float((ys[start_index]+.5)*resolution), float(rng.uniform(-np.pi, np.pi))]
        cfg['environment_randomization'] = dict(physical_layout_seed=actual_seed,
            outer_boundary='PDF20.15x11.35m', fresh_training_map=True,
            randomized=['obstacles', 'mounts', 'ids', 'marker_sizes', 'start_position', 'start_yaw'])
        if valid(cfg):
            return cfg
    raise RuntimeError(f'No accessible fresh map for {seed}/{difficulty}/{profile}')


def immutable_file(path, content):
    if path.exists() and path.read_text() != content:
        raise RuntimeError(f'Existing prepared artifact changed: {path}')
    path.write_text(content)


def plan(root, config, state):
    phase = state['phase']
    generation = state['generation']
    profiles = ('walls', 'mixed', 'obstacles', 'hidden_obstacles')
    difficulties = (8, 12, 16, 24, 32)
    if phase == 'train':
        parameters = population(state, config['seed'])
        specs = [(str(c), weights, level, i)
                 for i, level in enumerate(difficulties)
                 for c, weights in enumerate(parameters)]
    elif phase in ('pilot', 'validation'):
        count = 5 if phase == 'pilot' else 20
        specs = [(role, state['champion'] if role == 'candidate' else config['baseline_weights'],
                  difficulties[i % 5], i)
                 for i in range(count) for role in ('candidate', 'baseline')]
    elif not state.get('reserved_holdout_consumed', False):
        reserved = json.loads(Path(config['reserved_manifest']).read_text())
        specs = [('candidate', state['champion'], entry, i)
                 for i, entry in enumerate(reserved['episodes'])]
    else:
        specs = [('candidate', state['champion'], difficulties[i % 5], i)
                 for i in range(40)]
    # A phase is planned atomically. Deterministic files allow recovery if a
    # process dies after writing artifacts but before committing the state.
    for role, weights, difficulty, scene_index in specs:
        number = len(state['jobs'])
        directory = root / 'jobs' / f'{number:05d}_{phase}_g{generation}_{role}'
        directory.mkdir(parents=True, exist_ok=True)
        if phase == 'holdout' and isinstance(difficulty, dict):
            scene = difficulty
        elif role == 'baseline':
            scene = state['jobs'][-1]['scenario']
        else:
            cfg = fresh_scene(config['seed'] + number * 101, difficulty,
                              profiles[(generation + scene_index) % len(profiles)])
            file = root / 'scenarios' / f'{cfg["seed"]}.yaml'
            immutable_file(file, yaml.safe_dump(cfg, sort_keys=False))
            scene = dict(file=str(file), seed=cfg['seed'], sha256=digest(file),
                         requested_obstacles=difficulty, actual_obstacles=len(cfg['obstacles']),
                         mount_profile=cfg['mount_profile'])
            physical = {key: cfg[key] for key in ('start', 'obstacles', 'targets')}
            scene['physical_sha256'] = hashlib.sha256(json.dumps(physical, sort_keys=True).encode()).hexdigest()
            if phase == 'train' and any(j['phase'] == 'train' and
                    j['scenario'].get('physical_sha256') == scene['physical_sha256'] for j in state['jobs']):
                raise RuntimeError('Training map geometry was already used.')
        policy = dict(planner='edge', parameters=weights,
                      status='fresh_map_native_training_candidate', generation=generation)
        immutable_file(directory / 'policy.json', json.dumps(policy, indent=2) + '\n')
        manifest = dict(environment='gazebo_harmonic', planner='edge', camera_width=960,
                        duration=600, render_backend='software', start_docked=False,
                        approach_radius_m=1.5, protocol_version=2, record_video=True,
                        policy_sha256=digest(directory / 'policy.json'), scenarios=[scene])
        immutable_file(directory / 'manifest.json', json.dumps(manifest, indent=2) + '\n')
        state['jobs'].append(dict(number=number, phase=phase, generation=generation, role=role,
            parameters=weights, scenario=scene, output=str(directory), status='prepared',
            workspace=config['baseline_workspace'] if role == 'baseline' else config['workspace'],
            unit=f'osp-freshmap-{config["run_id"]}-{number:05d}.service'))
    state['planned'] = True


def finish_phase(root, config, state):
    jobs = [j for j in state['jobs'] if j['generation'] == state['generation']
            and j['phase'] == state['phase']]
    if state['phase'] == 'train':
        rows = []
        for role in ('0', '1', '2', '3'):
            selected = [j for j in jobs if j['role'] == role]
            rows.append(dict(role=role, parameters=selected[0]['parameters'],
                             **quality([j['outcome'] for j in selected])))
        rows.sort(key=rank_key, reverse=True)
        elite = np.log([row['parameters'] for row in rows[:2]])
        state['mean'] = (.4*np.array(state['mean']) + .6*elite.mean(axis=0)).tolist()
        state['std'] = np.maximum(.2, .5*np.array(state['std']) + .5*elite.std(axis=0)).tolist()
        state['champion'] = rows[0]['parameters']
        save(root / f'generation_{state["generation"]:04d}_ranking.json', rows)
        # Even poor rounds continue on fresh maps; pilot checks assess transfer.
        state['phase'] = 'pilot'
    elif state['phase'] in ('pilot', 'validation'):
        results = {role: quality([j['outcome'] for j in jobs if j['role'] == role])
                   for role in ('candidate', 'baseline')}
        save(root / f'generation_{state["generation"]:04d}_{state["phase"]}.json', results)
        count = 5 if state['phase'] == 'pilot' else 20
        minimum_success = .8 if state['phase'] == 'pilot' else .9
        if materially_better(results['candidate'], results['baseline'], count, minimum_success):
            state['phase'] = 'validation' if state['phase'] == 'pilot' else 'holdout'
            if state['phase'] == 'holdout':
                save(root / 'qualified_candidate.json', dict(parameters=state['champion'], paired_validation=results))
        else:
            state['generation'] += 1
            state['phase'] = 'train'
    else:
        results = quality([j['outcome'] for j in jobs])
        save(root / f'generation_{state["generation"]:04d}_holdout.json', results)
        state['reserved_holdout_consumed'] = True
        if results['episodes'] == 40 and results['success_rate'] >= .9 and results['contacts'] == 0:
            state['status'] = 'candidate_ready_for_completion_audit'
            save(root / 'candidate_policy.json', dict(planner='edge', parameters=state['champion'],
                 workspace=config['workspace'], status=state['status'], holdout=results))
        else:
            # A failed final test never becomes training data or gets reused.
            state['generation'] += 1
            state['phase'] = 'train'
    state['planned'] = False


def collect(job):
    directory = Path(job['output'])
    report = directory / 'report.json'
    outcome = dict(success=False, targets=0, approached=0, contacts=0,
                   seconds=600., infrastructure_failure=True, reason='missing_final_report')
    if report.exists():
        rows = json.loads(report.read_text()).get('rows', [])
        if len(rows) == 1:
            result = rows[0]['result']
            episode = directory / 'episodes' / str(job['scenario']['seed'])
            try:
                audited = audit_episode(episode, job['scenario'])
            except (OSError, ValueError, KeyError) as exc:
                audited = dict(success=False, errors=[repr(exc)])
            save(directory / 'episode_audit.json', audited)
            outcome.update(success=bool(result.get('success') and audited['success']),
                targets=result.get('targets') or 0, seconds=result.get('elapsed_seconds') or 600.,
                contacts=result.get('collisions') or 0, reason=result.get('reason'),
                approached=result.get('ground_truth_audit', {}).get('approached_within_1_5m', 0),
                infrastructure_failure=result.get('reason') not in ('all_targets_found', 'timeout'))
    job.update(status='done', outcome=outcome,
               completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())


def run(root):
    config = json.loads((root / 'configuration.json').read_text())
    lock = (root / 'coordinator.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    state_path = root / 'state.json'
    state = json.loads(state_path.read_text())
    while state['status'] == 'running':
        for workspace, expected in config['source_hashes'].items():
            if source_hashes(workspace) != expected:
                raise RuntimeError(f'Frozen runtime changed: {workspace}')
        for name, expected in config['driver_hashes'].items():
            if digest(Path(name)) != expected:
                raise RuntimeError(f'Frozen driver changed: {name}')
        for job in state['jobs']:
            metadata = Path(job['output']) / 'service.json'
            if job['status'] == 'prepared' and metadata.exists():
                job['status'] = 'running'
            if job['status'] == 'running' and not service_active(job['unit']):
                collect(job)
                save(state_path, state)
                print(json.dumps(dict(completed=job['number'], seed=job['scenario']['seed'],
                    phase=job['phase'], generation=job['generation'], **job['outcome'])), flush=True)
        completed = sorted((j for j in state['jobs'] if j['status'] == 'done'),
                           key=lambda j: j['completed_utc'])
        save(root / 'learning_log.json', [dict(number=j['number'], generation=j['generation'],
            phase=j['phase'], role=j['role'], scenario=j['scenario'],
            parameters=j['parameters'], outcome=j['outcome'],
            marker_events=str(Path(j['output'])/'episodes'/str(j['scenario']['seed'])/'marker_events.jsonl'),
            independent_audit=str(Path(j['output'])/'episode_audit.json')) for j in completed])
        recent = completed[-3:]
        if len(recent) == 3 and all(j['outcome']['infrastructure_failure'] for j in recent):
            state['status'] = 'needs_infrastructure_repair'
            save(state_path, state)
            break
        if not state['planned']:
            plan(root, config, state)
            save(state_path, state)
        current = [j for j in state['jobs'] if j['generation'] == state['generation']
                   and j['phase'] == state['phase']]
        if current and all(j['status'] == 'done' for j in current):
            finish_phase(root, config, state)
            save(state_path, state)
            continue
        if shutil.disk_usage(root).free < 8*1024**3:
            state['status'] = 'needs_disk_space'
            save(state_path, state)
            break
        active = [j for j in state['jobs'] if j['status'] == 'running']
        slots = max(0, 2-max(len(active), len(native_processes())))
        domains = {j.get('domain') for j in active}
        for job in [j for j in current if j['status'] == 'prepared'][:slots]:
            domain = next(d for d in (114, 115) if d not in domains)
            domains.add(domain)
            job['domain'] = domain
            command = [sys.executable, str(HERE / 'run_native_service.py'),
                '--unit', job['unit'].removesuffix('.service'), '--workspace', job['workspace'],
                '--output', job['output'], '--', '/usr/bin/python3', str(HERE / 'gazebo_benchmark.py'),
                '--output', job['output'], '--run-prepared', '--domain', str(domain), '--record-video']
            result = subprocess.run(command, capture_output=True, text=True)
            (Path(job['output']) / 'service_launch.log').write_text(result.stdout + result.stderr)
            if result.returncode:
                raise RuntimeError(f'Native launch failed for {job["number"]}. Inspect service metadata.')
            job['status'] = 'running'
            save(state_path, state)
        time.sleep(5.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    run(args.root.resolve())


if __name__ == '__main__':
    main()
