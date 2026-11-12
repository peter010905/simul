"""Insert one profiled native witness regression without changing queued sources."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time

from continual_native_curriculum import digest, save, source_hashes
from native_tournament import service_active
from observe_spatial_campaign import native_pid
from run_one_slot_reacquisition import resume_parent
from run_strict_approach_regression import run as run_profiled_case

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
UNIT = 'osp-windowed-witness-v1'


def require_settled_native_claims(parent, screen):
    # A transient service can still be sourcing its shell before exec. Do not
    # mistake that accepted launch for a free slot after stopping a dispatcher.
    while True:
        pending = False
        for root in (parent, screen):
            for job in json.loads((root/'state.json').read_text()).get('jobs', []):
                if (service_active(job['unit']) and native_pid(job['unit'], Path(job['output'])) is None
                        and service_active(job['unit'])):
                    pending = True
        if not pending: return
        time.sleep(.25)


def resume_screen(screen):
    state = json.loads((screen/'state.json').read_text())
    if state['status'] != 'running': return
    unit = state['coordinator_unit']
    if service_active(unit): return
    loaded = subprocess.run(['systemctl', '--user', 'show', unit, '-p', 'LoadState', '--value'],
        capture_output=True, text=True, check=True).stdout.strip()
    if loaded == 'loaded':
        result = subprocess.run(['systemctl', '--user', 'start', unit], capture_output=True, text=True)
        if result.returncode == 0 and service_active(unit): return
    subprocess.run(['systemd-run', '--user', '--unit='+unit.removesuffix('.service'),
        f'--property=WorkingDirectory={ROOT}', '--property=MemoryMax=512M', '--property=Restart=no',
        f'--property=StandardOutput=append:{screen/"coordinator.log"}',
        f'--property=StandardError=append:{screen/"coordinator.log"}',
        '/usr/bin/python3', '-u', str(HERE/'run_spatial_calibrated_screen.py'),
        '--parent', state['parent_campaign'], '--regression', state['prerequisite'],
        '--workspace', state['workspace'], '--output', str(screen)],
        capture_output=True, text=True, check=True)
    if not service_active(unit): raise RuntimeError('Original seven-shape screen did not resume')


def prepare(parent, screen, workspace, output):
    reference = json.loads((screen/'screen_0/state.json').read_text())['jobs'][0]
    old, new = source_hashes(Path(reference['workspace'])), source_hashes(workspace)
    changes = {k for k in old.keys()|new.keys() if old.get(k) != new.get(k)}
    if changes != {'osp_mission/marker_approach.py', 'test/test_marker_approach.py', 'test/test_windowed_witness_runtime.py'}:
        raise ValueError(f'Unexpected witness changes: {changes}')
    if '259 passed' not in (workspace.parent/'build_and_tests.log').read_text():
        raise ValueError('Require complete new runtime tests')
    proof = json.loads((ROOT/'repository_snapshot/witness_dropout_diagnosis.json').read_text())
    if (proof['baseline']['verified'] or not proof['candidate']['verified']
            or not proof['independent_physical_audit']['all_three_actual_approaches_verified']):
        raise ValueError('Require measured dropout reproduction with independent physical proof')
    output.mkdir(parents=True, exist_ok=False); case = output/'case_0'; case.mkdir()
    shutil.copy2(Path(reference['output'])/'policy.json', case/'policy.json')
    manifest = json.loads((Path(reference['output'])/'manifest.json').read_text())
    manifest['policy_sha256'] = digest(case/'policy.json'); save(case/'manifest.json', manifest)
    save(output/'state.json', dict(status='running', phase='diagnostic', training_samples=False,
        generalization_verified=False, production_promoted=False, parent_campaign=str(parent),
        coordinator_unit=UNIT+'.service', preserved_screen=str(screen),
        reference_output=reference['output'], source_hashes=new,
        source_change='Three individually valid near frames within1s;missing auxiliary fits do not erase prior valid frames',
        jobs=[dict(number=0, generation=0, phase='diagnostic', role='candidate', workspace=str(workspace),
            output=str(case), scenario=reference['scenario'], parameters=reference['parameters'],
            status='prepared', domain=122, unit=UNIT+'-native.service')]))


def run(parent, screen, workspace, output):
    if not (output/'state.json').exists(): prepare(parent, screen, workspace, output)
    state = json.loads((output/'state.json').read_text())
    if state['jobs'][0]['status'] == 'done': return
    original = json.loads((screen/'state.json').read_text())
    if original['status'] != 'running': raise RuntimeError('Require an active original screen to preserve')
    subprocess.run(['systemctl', '--user', 'stop', original['coordinator_unit']], check=True)
    stop = threading.Event()
    def restore():
        while not stop.wait(2):
            try:
                job = json.loads((output/'state.json').read_text())['jobs'][0]
                pid = native_pid(job['unit'], Path(job['output'])) if job['status'] == 'running' else None
                if pid is not None or job['status'] == 'done':
                    resume_screen(screen)
                    save(output/'screen_handoff.json', dict(status='original_screen_resumed',
                        verified_new_native_pid=pid, trigger='native_claim' if pid else 'terminal_cleanup',
                        original_native_and_queued_scenes_preserved=True))
                    return
            except Exception as exc:
                save(output/'screen_handoff.json', dict(status='resume_retry', error=repr(exc)))
    thread = threading.Thread(target=restore, daemon=True); thread.start()
    try:
        owner = json.loads((parent/'state.json').read_text())['coordinator_unit']
        subprocess.run(['systemctl', '--user', 'stop', owner], check=True)
        require_settled_native_claims(parent, screen)
        run_profiled_case(parent, screen, workspace, output)
    finally:
        stop.set(); thread.join(timeout=10)
        require_settled_native_claims(parent, screen)
        resume_parent(parent); resume_screen(screen)
    state = json.loads((output/'state.json').read_text())
    state['status'] = 'completed_windowed_witness_regression'; save(output/'state.json', state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('parent', 'screen', 'workspace', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.parent.resolve(), args.screen.resolve(), args.workspace.resolve(), args.output.resolve())
