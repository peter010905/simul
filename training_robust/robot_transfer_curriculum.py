"""Fixed-source native CEM with synthetic sensor stress and odometry memory.

Training maps are unique. Only paired qualification reuses a map across the
candidate and frozen calibrated predecessor. Both arms retain the full rubric.
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
from audit_approach_evidence import audit as audit_credits
from native_tournament import native_processes, service_active
from observe_spatial_campaign import native_pid
from probe_velocity_profile import probe as probe_velocity, gate as velocity_gate
from probe_encoder_delta_profile import probe as probe_encoder, gate as encoder_gate
from run_encoder_delta_regression import inspect_slam_transport
from run_floor_pose_regression import floor_gate, probe
from runtime_configuration_v2 import probe as probe_configuration, configuration_gate

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / 'training_robust'


def motion_profile(job):
    return probe_encoder,encoder_gate


def candidate_profiles(directory):
    path=Path(directory)/'report.json'
    if not path.exists():return False
    rows=json.loads(path.read_text()).get('rows',[])
    if len(rows)!=1:return False
    result=rows[0].get('result',{})
    return (result.get('marker_directed_look',{}).get('version')==1
        and result.get('face_directed_look',{}).get('version')==1
        and result.get('face_directed_look',{}).get('creates_coverage_or_recognition_credit') is False
        and result.get('marker_focus_profile',{}).get('desired_edge_pixels')==36.
        and result.get('near_pass_inspection',{}).get('maximum_looks')==8
        and result.get('near_pass_inspection',{}).get('maximum_sim_s')==35.
        and result.get('graph_landmark_update',{}).get('ground_truth_inputs') is False)


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


def fresh_scene(seed, difficulty, profile, shape):
    # These data are consumed by the simulator/evaluator, never by the policy.
    sys.path.insert(0, str(ROOT / 'src/osp_mission'))
    from osp_mission.challenge_scenarios import make_challenge_scene
    scene=make_challenge_scene(seed, difficulty, profile, shape)
    # Training starts vary in heading; no pose or target truth enters the policy.
    scene['start'][2]=float(np.random.default_rng(seed+71).uniform(-np.pi,np.pi))
    return scene


def immutable_file(path, content):
    if path.exists() and path.read_text() != content:
        raise RuntimeError(f'Existing prepared artifact changed: {path}')
    path.write_text(content)


def training_workspace(config, state, phase, role):
    if role == 'baseline':
        return config['baseline_workspace']
    return config['workspace']


def occupied_slots(jobs):
    """Include owned accepted shells that have not exec'd the benchmark yet."""
    actual = set(native_processes())
    pending = 0
    for job in jobs:
        if job['status'] != 'running': continue
        try: pid = native_pid(job['unit'], Path(job['output']))
        except subprocess.TimeoutExpired: pid = None
        if pid not in actual:
            pending += 1
    return len(actual)+pending


def benchmark_output_live(output):
    for pid in native_processes():
        try: argv = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
        except OSError: continue
        if str(output) in argv: return True
    return False


def strict_credit_gate(report):
    rows = report.get('rows', [])
    return bool(report.get('all_three_physical_credits_verified')
        and report.get('all_three_recognition_and_approach_events_logged')
        and len(rows) == 3 and all(row.get('range_witness_verified') for row in rows))


def observe(probe_function, job, filename):
    """A read-only observer failure must not kill or restart a native mission."""
    output = Path(job['output'])/filename
    try:
        return probe_function(job['domain'], output)
    except (subprocess.TimeoutExpired, subprocess.CalledProcessError, OSError, ValueError) as exc:
        save(output.with_suffix('.observation_error.json'), dict(
            error=repr(exc), native_not_restarted=True,
            utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
        return None


def plan(root, config, state):
    phase = state['phase']
    generation = state['generation']
    profiles = ('hidden_obstacles',)
    shapes = ('c_shape', 'u_shape', 'zigzag')
    difficulties = (8, 12, 16)
    if phase == 'train':
        parameters = population(state, config['seed'])
        specs = [(str(c), weights, difficulties[(i+generation) % len(difficulties)], i)
                 for c, weights in enumerate(parameters) for i in range(len(shapes))]
    elif phase in ('pilot', 'validation'):
        count = 3 if phase == 'pilot' else 12
        specs = [(role, state['champion'] if role == 'candidate' else config['baseline_weights'],
                  difficulties[i % len(difficulties)], i)
                 for i in range(count) for role in ('candidate', 'baseline')]
    elif phase == 'spatial_holdout':
        specs = [('candidate', state['champion'], difficulties[i % len(difficulties)], i) for i in range(18)]
    elif not state.get('reserved_holdout_consumed', False):
        reserved = json.loads(Path(config['reserved_manifest']).read_text())
        specs = [('candidate', state['champion'], entry, i)
                 for i, entry in enumerate(reserved['episodes'])]
    else:
        specs = [('candidate', state['champion'], difficulties[i % len(difficulties)], i)
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
            if phase == 'holdout':
                from continual_native_curriculum import fresh_scene as pdf_scene
                cfg = pdf_scene(config['seed'] + number * 101, difficulty,
                                ('walls','mixed','obstacles','hidden_obstacles')[(generation+scene_index)%4])
            else:
                cfg = fresh_scene(config['seed'] + number * 101, difficulty,
                                  profiles[(generation + scene_index) % len(profiles)],
                                  shapes[scene_index % len(shapes)])
            file = root / 'scenarios' / f'{cfg["seed"]}.yaml'
            immutable_file(file, yaml.safe_dump(cfg, sort_keys=False))
            scene = dict(file=str(file), seed=cfg['seed'], sha256=digest(file),
                         requested_obstacles=difficulty, actual_obstacles=len(cfg['obstacles']),
                         mount_profile=cfg['mount_profile'], arena={k:cfg.get('arena',dict(
                             shape='pdf_original',width_m=20.15,height_m=11.35,area_m2=20.15*11.35))[k]
                             for k in ('shape','width_m','height_m','area_m2')})
            physical = {key: cfg[key] for key in ('arena', 'start', 'obstacles', 'targets') if key in cfg}
            scene['physical_sha256'] = hashlib.sha256(json.dumps(physical, sort_keys=True).encode()).hexdigest()
            if phase == 'train' and any(j['phase'] == 'train' and
                    j['scenario'].get('physical_sha256') == scene['physical_sha256'] for j in state['jobs']):
                raise RuntimeError('Training map geometry was already used.')
        stress_profile=(state['jobs'][-1]['stress_profile'] if role=='baseline' else
            ('none','mild','moderate')[(scene_index+generation)%3])
        stress_seed=(state['jobs'][-1]['stress_seed'] if role=='baseline' else config['seed']+number*17+7)
        policy = dict(planner='edge', parameters=weights,
                      status='fresh_map_native_training_candidate', generation=generation)
        immutable_file(directory / 'policy.json', json.dumps(policy, indent=2) + '\n')
        manifest = dict(environment='gazebo_harmonic', planner='edge', camera_width=960,
                        duration=600, render_backend='software', start_docked=False,
                        approach_radius_m=1.5, protocol_version=2, record_video=True,
                        synthetic_stress_profile=stress_profile,synthetic_stress_seed=stress_seed,
                        policy_sha256=digest(directory / 'policy.json'), scenarios=[scene])
        immutable_file(directory / 'manifest.json', json.dumps(manifest, indent=2) + '\n')
        state['jobs'].append(dict(number=number, phase=phase, generation=generation, role=role,
            parameters=weights, scenario=scene, output=str(directory), status='prepared',
            stress_profile=stress_profile,stress_seed=stress_seed,
            workspace=training_workspace(config,state,phase,role),
            unit=f'osp-spatial-{config["run_id"]}-{number:05d}.service'))
    state['planned'] = True


def finish_phase(root, config, state):
    jobs = [j for j in state['jobs'] if j['generation'] == state['generation']
            and j['phase'] == state['phase']]
    if state['phase'] == 'train':
        rows = []
        for role in ('0', '1', '2', '3'):
            selected = [j for j in jobs if j['role'] == role]
            if len({j['workspace'] for j in selected})!=1:
                raise ValueError('Every coefficient candidate must use one immutable runtime')
            rows.append(dict(role=role, parameters=selected[0]['parameters'], workspace=selected[0]['workspace'],
                             **quality([j['outcome'] for j in selected])))
        rows.sort(key=rank_key, reverse=True)
        elite = np.log([row['parameters'] for row in rows[:2]])
        state['mean'] = (.4*np.array(state['mean']) + .6*elite.mean(axis=0)).tolist()
        state['std'] = np.maximum(.2, .5*np.array(state['std']) + .5*elite.std(axis=0)).tolist()
        state['champion'] = rows[0]['parameters']
        state['champion_workspace'] = rows[0]['workspace']
        save(root / f'generation_{state["generation"]:04d}_ranking.json', rows)
        # Even poor rounds continue on fresh maps; pilot checks assess transfer.
        state['phase'] = 'pilot'
    elif state['phase'] in ('pilot', 'validation'):
        results = {role: quality([j['outcome'] for j in jobs if j['role'] == role])
                   for role in ('candidate', 'baseline')}
        save(root / f'generation_{state["generation"]:04d}_{state["phase"]}.json', results)
        count = 3 if state['phase'] == 'pilot' else 12
        minimum_success = 1. if state['phase'] == 'pilot' else .9
        groups = shape_quality([j for j in jobs if j['role'] == 'candidate'])
        save(root / f'generation_{state["generation"]:04d}_{state["phase"]}_by_shape.json', groups)
        per_shape = all(row['success_rate'] >= .75 for row in groups.values()) and len(groups) == 3
        if per_shape and materially_better(results['candidate'], results['baseline'], count, minimum_success):
            state['phase'] = 'validation' if state['phase'] == 'pilot' else 'spatial_holdout'
            if state['phase'] == 'spatial_holdout':
                save(root / 'qualified_candidate.json', dict(parameters=state['champion'], workspace=state['champion_workspace'], paired_validation=results))
        else:
            state['generation'] += 1
            state['phase'] = 'train'
    elif state['phase'] == 'spatial_holdout':
        results = quality([j['outcome'] for j in jobs])
        groups = shape_quality(jobs)
        save(root / f'generation_{state["generation"]:04d}_spatial_holdout.json',
             dict(overall=results, by_shape=groups))
        state['spatial_holdout_consumed'] = True
        if results['episodes'] == 18 and results['success_rate'] >= .9 and results['contacts'] == 0 and len(groups) == 3 and all(
                group['episodes'] == 6 and group['successes'] >= 5 for group in groups.values()):
            state['phase'] = 'holdout'
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
                 workspace=state['champion_workspace'], status=state['status'], holdout=results))
        else:
            # A failed final test never becomes training data or gets reused.
            state['generation'] += 1
            state['phase'] = 'train'
    state['planned'] = False


def shape_quality(jobs):
    groups = {}
    for job in jobs:
        name = job['scenario'].get('arena', {}).get('shape', 'pdf_original')
        groups.setdefault(name, []).append(job['outcome'])
    return {name: quality(outcomes) for name, outcomes in groups.items()}


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
            if outcome['success']:
                expected_window = job['role'] == 'baseline' or result.get('approach',{}).get('witness_window_sim_s') == 1.0
                job['witness_algorithm_gate'] = expected_window
                if not expected_window:
                    outcome.update(success=False,infrastructure_failure=True,reason='windowed_witness_algorithm_not_observed')
                try:
                    physical = audit_credits(episode, Path(job['scenario']['file']))
                    save(directory/'physical_credit_audit.json', physical)
                    job['physical_credit_gate'] = strict_credit_gate(physical)
                    if not job['physical_credit_gate']:
                        job['raw_outcome_before_physical_gate'] = dict(outcome)
                        outcome.update(success=False, reason='physical_credit_not_verified')
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    save(directory/'physical_credit_audit_error.json', dict(error=repr(exc)))
                    job['raw_outcome_before_physical_gate'] = dict(outcome)
                    outcome.update(success=False, infrastructure_failure=True, reason='physical_credit_evidence_unavailable')
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
            if (job['status'] == 'running' and not service_active(job['unit'])
                    and not benchmark_output_live(job['output'])):
                collect(job)
                if job['role']!='baseline':
                    job['candidate_output_profiles_gate']=candidate_profiles(job['output'])
                    if not (job['candidate_output_profiles_gate'] and job.get('slam_transport_gate')):
                        job['raw_outcome_before_candidate_profile_gate']=dict(job['outcome'])
                        job['outcome'].update(success=False,infrastructure_failure=True,reason='face_runtime_profiles_unverified')
                if (job.get('floor_sensor_gate') is not True or job.get('runtime_configuration_gate') is not True
                        or job.get('velocity_profile_gate') is not True):
                    job['raw_outcome_before_floor_gate'] = dict(job['outcome'])
                    job['outcome'].update(success=False,infrastructure_failure=True,
                        reason='native_input_or_effective_motion_configuration_unverified')
                if job.get('transfer_input_gate') is not True:
                    job['raw_outcome_before_transfer_gate']=dict(job['outcome'])
                    job['outcome'].update(success=False,infrastructure_failure=True,reason='synthetic_stress_input_unverified')
                save(state_path, state)
                print(json.dumps(dict(completed=job['number'], seed=job['scenario']['seed'],
                    phase=job['phase'], generation=job['generation'], **job['outcome'])), flush=True)
        completed = sorted((j for j in state['jobs'] if j['status'] == 'done'),
                           key=lambda j: j['completed_utc'])
        save(root / 'learning_log.json', [dict(number=j['number'], generation=j['generation'],
            phase=j['phase'], role=j['role'], workspace=j['workspace'], scenario=j['scenario'],
            parameters=j['parameters'], outcome=j['outcome'],
            marker_events=str(Path(j['output'])/'episodes'/str(j['scenario']['seed'])/'marker_events.jsonl'),
                independent_audit=str(Path(j['output'])/'episode_audit.json'),
                physical_credit_audit=str(Path(j['output'])/'physical_credit_audit.json')) for j in completed])
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
        for job in [j for j in active if 'floor_sensor_gate' not in j]:
            live = Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if not live.exists():
                continue
            current_live = json.loads(live.read_text()).get('status',{})
            if current_live.get('state') != 'running':
                continue
            report = observe(probe, job, 'floor_probe.json')
            if report is None: continue
            names = ('cliff_front_left','cliff_front_right','cliff_side_left','cliff_side_right')
            complete = (report.get('samples',{}).get('hazard_frames',0)>0
                and all(report.get('samples',{}).get(name,0)>=3 for name in names))
            if not complete: continue
            job['floor_sensor_gate'] = floor_gate(report)
            save(state_path,state)
            if not job['floor_sensor_gate']:
                state['status'] = 'needs_floor_sensor_repair'
                save(state_path,state)
                raise RuntimeError(f'Invalid floor sensor input in job{job["number"]}; preserve native evidence and repair the environment')
        for job in [j for j in active if j.get('floor_sensor_gate') and not j.get('runtime_configuration_gate')]:
            measured=observe(probe_configuration,job,'runtime_configuration.json')
            if measured is None: continue
            expected_ack=config['baseline_action_acknowledgement_timeout_ms'] if job['role']=='baseline' else 1000
            live_local=config['baseline_live_local_costmap'] if job['role']=='baseline' else True
            check=configuration_gate(measured,Path(job['output'])/'policy.json',expected_ack,live_local=live_local)
            job['runtime_configuration_gate']=check['passed']
            job['configuration_observation_attempts']=job.get('configuration_observation_attempts',0)+1
            save(Path(job['output'])/'runtime_configuration_check.json',check)
            save(state_path,state)
            if check['mismatches']:
                state['status']='needs_effective_configuration_repair';save(state_path,state)
                raise RuntimeError(f'Observed ROS parameters differ from the frozen profile for job{job["number"]}')
            # A missing response is an observation gap, not a terminal mission.
            # Re-poll the same live run instead of restarting or changing it.
        for job in [j for j in active if j.get('runtime_configuration_gate') and (
                not j.get('velocity_profile_gate') or j['role']!='baseline' and not j.get('slam_transport_gate'))]:
            live = Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
            if not live.exists() or json.loads(live.read_text()).get('status',{}).get('state') != 'running': continue
            if job['role']!='baseline' and not job.get('slam_transport_gate'):
                try:pid=native_pid(job['unit'],Path(job['output']))
                except subprocess.TimeoutExpired as exc:
                    save(Path(job['output'])/'slam_transport_observation_gap.json',dict(error=repr(exc),native_not_restarted=True))
                    pid=None
                measured=inspect_slam_transport(pid) if pid else {}
                save(Path(job['output'])/'effective_slam_transport.json',measured)
                job['slam_transport_gate']=bool(measured.get('server_environment',{}).get('FASTDDS_BUILTIN_TRANSPORTS')=='UDPv4' and measured.get('shm_threads')==[])
            profile_probe,profile_gate=motion_profile(job)
            report = observe(profile_probe,job,'effective_velocity_profile.json')
            if report is None: continue
            job['velocity_profile_gate'] = profile_gate(report)
            job['velocity_observation_attempts'] = job.get('velocity_observation_attempts',0)+1
            save(state_path,state)
            if report.get('complete') and not job['velocity_profile_gate']:
                state['status']='needs_effective_motion_configuration_repair';save(state_path,state)
                raise RuntimeError('Observed encoder/EKF/SLAM profile differs from frozen source')
        for job in [j for j in active if j.get('velocity_profile_gate') and not j.get('transfer_input_gate')]:
            from probe_transfer_input import probe as transfer_probe, gate as transfer_gate
            measured=observe(transfer_probe,job,'transfer_input.json')
            if measured is not None:
                job['transfer_input_gate']=transfer_gate(measured,job['stress_profile'])
                save(state_path,state)
        slots = max(0, 2-occupied_slots(state['jobs']))
        domains = {j.get('domain') for j in active}
        # An older coordinator may have stopped dispatching while its last
        # native job drains. Count its actual ROS domain as well as its slot.
        for pid in native_processes():
            try:
                argv = (Path('/proc')/str(pid)/'cmdline').read_bytes().decode().split('\0')
                if '--domain' in argv:
                    domains.add(int(argv[argv.index('--domain')+1]))
            except (OSError, ValueError, IndexError):
                continue
        for job in [j for j in current if j['status'] == 'prepared'][:slots]:
            if occupied_slots(state['jobs']) >= 2: break
            domain = next((d for d in config['domains'] if d not in domains), None)
            if domain is None:
                break
            domains.add(domain)
            job['domain'] = domain
            command = [sys.executable, str(HERE / 'run_native_service.py'),
                '--unit', job['unit'].removesuffix('.service'), '--workspace', job['workspace'],
                '--output', job['output'], '--', '/usr/bin/env',
                'OSP_TRANSFER_PROFILE='+job['stress_profile'],'OSP_TRANSFER_SEED='+str(job['stress_seed']),
                '/usr/bin/python3', str(HERE / 'gazebo_benchmark.py'),
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
