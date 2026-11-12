"""Compare a staged perception change after the current native pair finishes.

The frozen CEM coordinator is stopped during this finite regression. Existing
Gazebo units finish normally. The original coordinator resumes in finally.
"""
import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from continual_native_curriculum import collect, digest, save, source_hashes
from native_tournament import native_processes, service_active

HERE = Path(__file__).resolve().parent
COORDINATOR = 'osp-fresh-map-cem-v1.service'


def resume_coordinator(campaign):
    result = subprocess.run(['systemctl', '--user', 'start', COORDINATOR],
                            capture_output=True, text=True)
    if result.returncode == 0:
        return
    loaded = subprocess.run(['systemctl', '--user', 'show', COORDINATOR,
                             '-p', 'LoadState', '--value'], capture_output=True, text=True)
    if loaded.stdout.strip() != 'not-found':
        raise RuntimeError(f'Cannot resume coordinator: {result.stderr}')
    # systemd discards stopped transient unit definitions. Recreate the same
    # bounded coordinator rather than restarting or relaunching native jobs.
    subprocess.run(['systemd-run', '--user', f'--unit={COORDINATOR}',
        f'--property=WorkingDirectory={HERE.parent}', '--property=MemoryMax=512M',
        '--property=Restart=no',
        f'--property=StandardOutput=append:{campaign/"coordinator.log"}',
        f'--property=StandardError=append:{campaign/"coordinator.log"}',
        '/usr/bin/python3', '-u', str(HERE/'continual_native_curriculum.py'), str(campaign)], check=True)


def run(campaign, workspace, output):
    output.mkdir(parents=True, exist_ok=True)
    state_file = output/'regression_state.json'
    if state_file.exists():
        raise RuntimeError('Preserve the existing regression; inspect its service handles before resuming.')
    state = dict(status='waiting_for_current_pair', regression_only=True,
                 training_samples=False, original_coordinator=COORDINATOR, jobs=[])
    save(state_file, state)
    subprocess.run(['systemctl', '--user', 'stop', COORDINATOR], check=True)
    try:
        original = json.loads((campaign/'state.json').read_text())
        baseline_jobs = original['jobs'][:2]
        state['waiting_units'] = [j['unit'] for j in original['jobs'] if j['status'] == 'running']
        state['candidate_source_hashes'] = source_hashes(workspace)
        save(state_file, state)
        while any(service_active(unit) for unit in state['waiting_units']):
            time.sleep(5.)
        # With the coordinator stopped, these state updates cannot race it.
        for job in original['jobs']:
            if job['status'] == 'running':
                collect(job)
        save(campaign/'state.json', original)
        for index, baseline in enumerate(baseline_jobs):
            directory = output/f'case_{index}'
            directory.mkdir(exist_ok=False)
            shutil.copy2(Path(baseline['output'])/'policy.json', directory/'policy.json')
            manifest = json.loads((Path(baseline['output'])/'manifest.json').read_text())
            manifest['policy_sha256'] = digest(directory/'policy.json')
            save(directory/'manifest.json', manifest)
            state['jobs'].append(dict(number=index, output=str(directory),
                scenario=baseline['scenario'], status='prepared',
                baseline=baseline['outcome'], unit=f'osp-{output.parent.name.replace("_", "-")}-{index}.service'))
        state['status'] = 'running_regression'
        save(state_file, state)
        while any(job['status'] != 'done' for job in state['jobs']):
            if source_hashes(workspace) != state['candidate_source_hashes']:
                raise RuntimeError('Frozen provisional-inspection candidate changed.')
            for job in state['jobs']:
                if job['status'] == 'running' and not service_active(job['unit']):
                    collect(job)
                    save(state_file, state)
            active = [job for job in state['jobs'] if job['status'] == 'running']
            slots = max(0, 2-max(len(native_processes()), len(active)))
            domains = {job.get('domain') for job in active}
            for job in [job for job in state['jobs'] if job['status'] == 'prepared'][:slots]:
                job['domain'] = next(domain for domain in (116, 117) if domain not in domains)
                domains.add(job['domain'])
                command = ['/usr/bin/python3', str(HERE/'run_native_service.py'),
                    '--unit', job['unit'].removesuffix('.service'), '--workspace', str(workspace),
                    '--output', job['output'], '--', '/usr/bin/python3', str(HERE/'gazebo_benchmark.py'),
                    '--output', job['output'], '--run-prepared', '--domain', str(job['domain']), '--record-video']
                subprocess.run(command, check=True, capture_output=True, text=True)
                job['status'] = 'running'
                save(state_file, state)
            time.sleep(5.)
        state['status'] = 'completed_regression_requires_fresh_map_validation'
        save(state_file, state)
        save(output/'comparison.json', [dict(seed=j['scenario']['seed'],
            baseline=j['baseline'], candidate=j['outcome']) for j in state['jobs']])
    except BaseException as exc:
        state.update(status='regression_error', error=repr(exc))
        save(state_file, state)
        raise
    finally:
        resume_coordinator(campaign)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', type=Path, required=True)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(143))
    run(args.campaign.resolve(), args.workspace.resolve(), args.output.resolve())


if __name__ == '__main__':
    main()
