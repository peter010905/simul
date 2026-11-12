"""Single-owner native insertion; handle already-unloaded dispatchers explicitly."""
import argparse
import json
from pathlib import Path
import subprocess
import threading
import time

from continual_native_curriculum import save
from native_tournament import service_active
from observe_spatial_campaign import native_pid
from probe_velocity_profile import gate, probe
from run_fresh_variant_probe import run as run_probe
from run_one_slot_reacquisition import resume_parent
from run_windowed_witness_regression import prepare as prepare_reference, resume_screen

UNIT = 'osp-windowed-witness-v2'


def settle_claims(*roots):
    while True:
        pending = False
        for root in roots:
            for job in json.loads((root/'state.json').read_text()).get('jobs', []):
                if (service_active(job['unit']) and native_pid(job['unit'], Path(job['output'])) is None
                        and service_active(job['unit'])):
                    pending = True
        if not pending: return
        time.sleep(.25)


def prepare(parent, screen, workspace, output):
    prepare_reference(parent, screen, workspace, output)
    state = json.loads((output/'state.json').read_text())
    state['coordinator_unit'] = UNIT+'.service'; state['jobs'][0]['unit'] = UNIT+'-native.service'
    state['prior_coordinator_failure'] = 'v1double_stop_on_unloaded_CEM;no_native_trial_started'
    save(output/'state.json', state)


def run(parent, screen, workspace, output):
    if not (output/'state.json').exists(): prepare(parent, screen, workspace, output)
    path = output/'state.json'; state = json.loads(path.read_text())
    if state['jobs'][0]['status'] == 'done': return
    original = json.loads((screen/'state.json').read_text())
    owner = json.loads((parent/'state.json').read_text())
    if original['status'] != 'running' or owner['status'] != 'running':
        raise RuntimeError('Require original screen and CEM state to preserve')
    stop = threading.Event()
    def restore():
        while not stop.wait(2):
            try:
                job = json.loads(path.read_text())['jobs'][0]
                pid = native_pid(job['unit'], Path(job['output'])) if job['status'] == 'running' else None
                if pid is not None or job['status'] == 'done':
                    resume_parent(parent); resume_screen(screen)
                    save(output/'dispatch_handoff.json', dict(status='original_dispatchers_resumed',
                        verified_new_native_pid=pid, trigger='native_claim' if pid else 'terminal_cleanup',
                        existing_native_and_queued_scenes_preserved=True))
                    return
            except Exception as exc:
                save(output/'dispatch_handoff.json', dict(status='resume_retry', error=repr(exc)))
    def measure():
        target = output/'effective_velocity_profile.json'
        while not stop.wait(3):
            if target.exists() and gate(json.loads(target.read_text())): return
            job = json.loads(path.read_text())['jobs'][0]
            if job['status'] == 'done': return
            try:
                live = Path(job['output'])/'episodes'/str(job['scenario']['seed'])/'live_status.json'
                if (job['status'] == 'running' and native_pid(job['unit'], Path(job['output'])) is not None
                        and live.exists() and json.loads(live.read_text()).get('status', {}).get('state') == 'running'):
                    if gate(probe(job['domain'], target)): return
            except Exception as exc:
                save(output/'profile_observation_error.json', dict(error=repr(exc), native_not_restarted=True))
    threads = [threading.Thread(target=target, daemon=True) for target in (restore, measure)]
    try:
        # Stop each dispatcher only when active. Transient inactive units may
        # already have been unloaded; no second owner issues another stop.
        for unit in (original['coordinator_unit'], owner['coordinator_unit']):
            if service_active(unit): subprocess.run(['systemctl', '--user', 'stop', unit], check=True)
        settle_claims(parent, screen, output)
        for thread in threads: thread.start()
        run_probe(argparse.Namespace(output=output, workspace=workspace))
    finally:
        stop.set()
        for thread in threads:
            if thread.ident is not None: thread.join(timeout=35)
        settle_claims(parent, screen, output)
        resume_parent(parent); resume_screen(screen)
    state = json.loads(path.read_text()); job = state['jobs'][0]
    profile = output/'effective_velocity_profile.json'
    state['velocity_profile_verified'] = profile.exists() and gate(json.loads(profile.read_text()))
    if not state['velocity_profile_verified']:
        job['raw_outcome_before_velocity_gate'] = dict(job['outcome'])
        job['outcome'].update(success=False, infrastructure_failure=True, reason='velocity_profile_unverified')
    state['status'] = 'completed_windowed_witness_regression'; save(path, state)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('parent', 'screen', 'workspace', 'output'): parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.parent.resolve(), args.screen.resolve(), args.workspace.resolve(), args.output.resolve())
