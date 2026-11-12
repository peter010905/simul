"""Fresh seven-shape engineering screen, gated by the physical-approach regression.

One native slot at a time alongside the original CEM. These are new training
spaces, never the reserved qualification/holdout set. Scores and videos remain
unaltered; exact-credit and input gates are recorded separately.
"""
import argparse
import copy
import json
from pathlib import Path
import threading
import time

from continual_native_curriculum import digest, save, source_hashes
from launch_spatial_curriculum import audit_geometry
from observe_spatial_campaign import native_pid
from probe_velocity_profile import gate, probe
from run_fresh_variant_probe import prepare as prepare_case
from run_one_slot_reacquisition import run as run_reserved

SHAPES = ('circle', 'triangle', 'trapezoid', 'rectangle', 'l_shape', 'u_shape', 'rooms')
UNIT = 'osp-spatial-calibrated-screen-v1'
HERE = Path(__file__).resolve().parent


def driver_hashes():
    return {str(HERE/name): digest(HERE/name) for name in (
        'run_spatial_calibrated_screen.py', 'run_fresh_variant_probe.py',
        'run_one_slot_reacquisition.py', 'spatial_native_curriculum_v5.py',
        'run_native_service.py', 'gazebo_benchmark.py')}


def qualified_regression(state, analysis):
    job = state['jobs'][0]
    physical = analysis.get('physical_credit_audit', {})
    rows = physical.get('rows', [])
    return bool(job['status'] == 'done' and job['outcome']['success']
        and state.get('velocity_profile_verified')
        and physical.get('all_three_physical_credits_verified')
        and physical.get('all_three_recognition_and_approach_events_logged')
        and len(rows) == 3 and all(r.get('range_witness_verified') for r in rows))


def prepare(parent, regression, workspace, output):
    if '253 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require the full combined runtime tests')
    prior = json.loads((regression/'state.json').read_text())
    before = source_hashes(Path(prior['jobs'][0]['workspace'])); after = source_hashes(workspace)
    changed = {key for key in before.keys()|after.keys() if before.get(key) != after.get(key)}
    if changed != {'osp_mission/edge_policy.py', 'osp_mission/mission.py',
            'test/test_bounded_gain_runtime.py', 'test/test_calibrated_gain_runtime.py'}:
        raise ValueError(f'Unexpected combined exploration changes: {changed}')
    used_seeds = set()
    for statefile in parent.parent.glob('*/state.json'):
        value = json.loads(statefile.read_text())
        used_seeds.update(j.get('scenario', {}).get('seed') for j in value.get('jobs', []))
    seeds = [1171300000 + i*101 for i in range(7)]
    if used_seeds.intersection(seeds): raise ValueError('Do not reuse a previous training seed')
    output.mkdir(parents=True, exist_ok=False)
    jobs = []
    policy = Path(prior['jobs'][0]['output'])/'policy.json'
    for number, (shape, obstacles, seed) in enumerate(zip(SHAPES, (12, 16, 24, 32, 8, 12, 16), seeds)):
        child = output/f'screen_{number}'
        prepare_case(argparse.Namespace(output=child, workspace=workspace, policy=policy,
            seed=seed, obstacles=obstacles, shape=shape, domain=120, unit=f'{UNIT}-{number}'))
        value = json.loads((child/'state.json').read_text())
        value['coordinator_unit'] = UNIT+'.service'; value['jobs'][0]['generation'] = 0
        save(child/'state.json', value)
        job = copy.deepcopy(value['jobs'][0]); job['number'] = number; jobs.append(job)
    save(output/'state.json', dict(status='running', phase='waiting_physical_regression',
        generation=0, coordinator_unit=UNIT+'.service', parent_campaign=str(parent),
        prerequisite=str(regression), workspace=str(workspace), source_hashes=after,
        driver_hashes=driver_hashes(),
        training_samples=True, engineering_screen=True, generalization_verified=False,
        production_promoted=False, reserved_holdout_consumed=False, spatial_holdout_consumed=False,
        maximum_total_native_benchmarks=2, jobs=jobs))
    audit_geometry(output)


def sync_job(state, number, child):
    job = copy.deepcopy(json.loads((child/'state.json').read_text())['jobs'][0])
    job['number'] = number
    state['jobs'][number] = job
    return job


def run(parent, regression, workspace, output):
    if not (output/'state.json').exists(): prepare(parent, regression, workspace, output)
    path = output/'state.json'; state = json.loads(path.read_text())
    if state['status'] != 'running': return
    try:
        terminal_seen = None
        while True:
            if driver_hashes() != state['driver_hashes']: raise RuntimeError('Frozen screen driver changed')
            prior = json.loads((regression/'state.json').read_text())
            job = prior['jobs'][0]
            if job['status'] == 'done' and prior['status'].startswith('completed_'):
                if terminal_seen is None: terminal_seen = time.monotonic()
                report = Path(job['output'])/'observer_analysis.json'
                if report.exists():
                    analysis = json.loads(report.read_text())
                    if not qualified_regression(prior, analysis):
                        state.update(status='not_started_prerequisite_failed',
                            prerequisite_outcome=job['outcome'],
                            prerequisite_physical_credit_audit=analysis.get('physical_credit_audit'))
                        save(path, state); return
                    break
                if time.monotonic()-terminal_seen > 120:
                    raise RuntimeError('Prerequisite finalized but its independent physical audit is missing')
            time.sleep(5)
        state['phase'] = 'fresh_shape_engineering_screen'; save(path, state)
        for number in range(7):
            child = output/f'screen_{number}'
            if state['jobs'][number]['status'] == 'done' and 'strict_screen_passed' in state['jobs'][number]: continue
            if source_hashes(workspace) != state['source_hashes']: raise RuntimeError('Frozen source changed')
            if driver_hashes() != state['driver_hashes']: raise RuntimeError('Frozen screen driver changed')
            errors = []
            def execute():
                try: run_reserved(parent, workspace, child)
                except BaseException as exc: errors.append(exc)
            worker = threading.Thread(target=execute); worker.start()
            try:
                while worker.is_alive():
                    job = sync_job(state, number, child); save(path, state)
                    effective = child/'effective_velocity_profile.json'
                    live = Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
                    if (not effective.exists() and job['status'] == 'running'
                            and native_pid(job['unit'], Path(job['output'])) is not None and live.exists()
                            and json.loads(live.read_text()).get('status', {}).get('state') == 'running'):
                        try: probe(job['domain'], effective)
                        except Exception as exc:
                            save(child/'profile_observation_error.json', {'error': repr(exc), 'native_not_restarted': True})
                    worker.join(timeout=5)
            finally:
                # A manager error must not abandon a child while another case
                # starts. The owner keeps the global two-native limit.
                worker.join()
            if errors: raise errors[0]
            job = sync_job(state, number, child); save(path, state)
            report = Path(job['output'])/'observer_analysis.json'
            deadline = time.monotonic()+90
            while not report.exists() and time.monotonic() < deadline: time.sleep(3)
            analysis = json.loads(report.read_text()) if report.exists() else {}
            physical = analysis.get('physical_credit_audit', {})
            effective = child/'effective_velocity_profile.json'
            profile_ok = effective.exists() and gate(json.loads(effective.read_text()))
            witnesses_ok = (len(physical.get('rows', [])) == 3
                and all(r.get('range_witness_verified') for r in physical['rows']))
            job.update(velocity_profile_verified=profile_ok,
                physical_credit_verified=physical.get('all_three_physical_credits_verified', False),
                all_range_witnesses_verified=witnesses_ok,
                strict_screen_passed=bool(job['outcome']['success'] and profile_ok and witnesses_ok
                    and physical.get('all_three_physical_credits_verified')
                    and physical.get('all_three_recognition_and_approach_events_logged')))
            save(path, state)
        outcomes = [j['outcome'] for j in state['jobs']]
        state.update(status='completed_engineering_screen',
            original_benchmark_successes=sum(o['success'] for o in outcomes),
            strict_successes=sum(j['strict_screen_passed'] for j in state['jobs']),
            strict_par2_seconds=sum(j['outcome']['seconds'] if j['strict_screen_passed'] else 1200
                for j in state['jobs'])/7)
        save(path, state)
    except BaseException as exc:
        state.update(status='engineering_screen_manager_error', error=repr(exc),
            original_native_evidence_preserved=True)
        save(path, state); raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('parent', 'regression', 'workspace', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.parent.resolve(), args.regression.resolve(), args.workspace.resolve(), args.output.resolve())
