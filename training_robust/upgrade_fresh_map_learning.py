"""Adopt audited improvements after native comparisons without interrupting runs."""
import argparse
import fcntl
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from continual_native_curriculum import HERE, collect, finish_phase, native_processes, plan, save, source_hashes
from audit_fresh_map_diversity import audit
from native_tournament import service_active
from prepare_fresh_map_successor import prepare, tf_decision
from run_small_marker_regression import resume_coordinator

ORIGINAL_SERVICE = 'osp-fresh-map-cem-v1.service'
NEW_SERVICE = 'osp-fresh-map-cem-v2.service'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original', type=Path, required=True)
    parser.add_argument('--perception-regression', type=Path, required=True)
    parser.add_argument('--perception-workspace', type=Path, required=True)
    parser.add_argument('--tf-regression', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    args.original = args.original.resolve()
    args.destination = args.destination.resolve()
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(143))
    record = args.original/'successor_transition.json'
    save(record, dict(status='waiting_for_tf_comparison', destination=str(args.destination)))
    while service_active('osp-tf-restamp-regression-v1.service'):
        time.sleep(5.)
    perception = json.loads((args.perception_regression/'regression_state.json').read_text())
    if (perception['status'] != 'completed_regression_requires_fresh_map_validation'
            or len(perception['jobs']) != 2 or not all(j['outcome']['success'] for j in perception['jobs'])):
        raise RuntimeError('Perception candidate did not pass both independent native audits.')
    if source_hashes(args.perception_workspace.resolve()) != perception['candidate_source_hashes']:
        raise RuntimeError('Audited perception source changed.')
    for job in perception['jobs']:
        audited = json.loads((Path(job['output'])/'episode_audit.json').read_text())
        if not audited['success'] or audited['errors']:
            raise RuntimeError('Independent perception audit is missing or failed.')
    original = json.loads((args.original/'state.json').read_text())
    decision = tf_decision(args.tf_regression, original)
    workspace = prepare(args.original, args.perception_workspace.resolve(), args.destination, decision)
    save(record, dict(status='building_successor', destination=str(args.destination), tf_selection=decision))
    command = ('source /opt/ros/jazzy/setup.bash && '
        'colcon build --packages-select osp_mission && source install/setup.bash && '
        'python3 -m pytest src/osp_mission/test -q')
    with (args.destination/'build_and_tests.log').open('w') as output:
        subprocess.run(['bash', '-c', command], cwd=workspace, stdout=output, stderr=subprocess.STDOUT, check=True)
    while True:
        original = json.loads((args.original/'state.json').read_text())
        if original['status'] != 'running':
            raise RuntimeError(f'Original search needs attention: {original["status"]}')
        active = [j for j in original['jobs'] if j['status'] == 'running' and service_active(j['unit'])]
        if len(active) == 2:
            break
        time.sleep(5.)
    stopped = False
    lock = None
    try:
        subprocess.run(['systemctl', '--user', 'stop', ORIGINAL_SERVICE], check=True)
        stopped = True
        lock = (args.original/'coordinator.lock').open('w')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        original = json.loads((args.original/'state.json').read_text())
        waiting = [j for j in original['jobs'] if j['status'] == 'running']
        save(record, dict(status='draining_original_jobs', units=[j['unit'] for j in waiting],
                          destination=str(args.destination), tf_selection=decision))
        while any(service_active(j['unit']) for j in waiting) or native_processes():
            time.sleep(5.)
        for job in waiting:
            collect(job)
        current = [job for job in original['jobs'] if job['generation'] == original['generation']
                   and job['phase'] == original['phase']]
        if original['phase'] == 'train' and current and all(job['status'] == 'done' for job in current):
            old_config = json.loads((args.original/'configuration.json').read_text())
            finish_phase(args.original, old_config, original)
        original.update(status='superseded_by_fresh_map_cem_v2', successor=str(args.destination))
        save(args.original/'state.json', original)
        # Preserve the most recent learned distribution after the original
        # native jobs drain. Trial scores from different runtime versions stay separate.
        next_state = json.loads((args.destination/'state.json').read_text())
        for key in ('champion', 'mean', 'std'):
            next_state[key] = original[key]
        next_state['reserved_holdout_consumed'] = (original.get('reserved_holdout_consumed', False)
            or any(job['phase'] == 'holdout' and job['status'] == 'done' for job in original['jobs']))
        config = json.loads((args.destination/'configuration.json').read_text())
        config['source_hashes'][str(workspace)] = source_hashes(workspace)
        save(args.destination/'configuration.json', config)
        plan(args.destination, config, next_state)
        save(args.destination/'state.json', next_state)
        diversity = audit(args.destination)
        previous_diversity = audit(args.original)
        previous_maps = {row['occupancy_sha256'] for row in previous_diversity['rows']}
        if (not diversity['no_reused_occupancy'] or any(row['occupancy_sha256'] in previous_maps
                for row in diversity['rows'])):
            raise RuntimeError('Successor must use different occupancy maps from the original curriculum.')
        save(args.destination/'initial_diversity_audit.json', diversity)
        save(args.original/'learning_log.json', [dict(number=job['number'], generation=job['generation'],
            phase=job['phase'], scenario=job['scenario'], parameters=job['parameters'], outcome=job['outcome'],
            marker_events=str(Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'marker_events.jsonl'))
            for job in original['jobs'] if job['status'] == 'done'])
        subprocess.run(['systemd-run', '--user', f'--unit={NEW_SERVICE}',
            f'--property=WorkingDirectory={HERE.parent}', '--property=MemoryMax=512M',
            '--property=Restart=no', f'--property=StandardOutput=append:{args.destination/"coordinator.log"}',
            f'--property=StandardError=append:{args.destination/"coordinator.log"}',
            '/usr/bin/python3', '-u', str(HERE/'continual_native_curriculum.py'), str(args.destination)], check=True)
        if not service_active(NEW_SERVICE):
            raise RuntimeError('Successor coordinator did not become active.')
        stopped = False
        save(record, dict(status='successor_active', service=NEW_SERVICE, destination=str(args.destination),
                          tf_selection=decision, goal_verified=False))
    except BaseException as exc:
        save(record, dict(status='transition_error', error=repr(exc), destination=str(args.destination)))
        if stopped and not service_active(NEW_SERVICE):
            original = json.loads((args.original/'state.json').read_text())
            original['status'] = 'running'
            save(args.original/'state.json', original)
            if lock is not None:
                lock.close()
                lock = None
            resume_coordinator(args.original)
        raise
    finally:
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    main()
