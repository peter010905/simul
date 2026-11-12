"""Exercise the unloaded-transient regression through the actual new owner flow."""
import json
import subprocess
from pathlib import Path

import pytest
import run_windowed_witness_regression_v2 as runner
from probe_velocity_profile import EXPECTED


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value))


@pytest.mark.parametrize('cem_active', [False, True])
def test_unloaded_cem_is_skipped_and_active_cem_is_stopped_exactly_once(tmp_path, monkeypatch, cem_active):
    parent, screen, workspace, output = [tmp_path/name for name in ('parent', 'screen', 'workspace', 'output')]
    put(parent/'state.json', dict(status='running', coordinator_unit='osp-test-cem.service', jobs=[]))
    put(screen/'state.json', dict(status='running', coordinator_unit='osp-test-screen.service', jobs=[]))
    put(output/'state.json', dict(status='running', jobs=[dict(status='prepared', number=0,
        unit='osp-test-native.service', output=str(output/'case'), domain=122, scenario={'seed': 1})]))
    put(output/'effective_velocity_profile.json', {'complete': True, 'parameters': EXPECTED})
    active = {'osp-test-cem.service': cem_active, 'osp-test-screen.service': True,
        'osp-test-native.service': False}; commands = []; launches = []; restored = []
    monkeypatch.setattr(runner, 'service_active', lambda unit: active.get(unit, False))
    def systemd(command, **kwargs):
        assert command[:3] == ['systemctl', '--user', 'stop']
        unit = command[-1]
        # Reproduce the real status5 error if any owner double-stops an unloaded unit.
        if not active[unit]: raise subprocess.CalledProcessError(5, command)
        active[unit] = False; commands.append(command)
    monkeypatch.setattr(runner.subprocess, 'run', systemd)
    monkeypatch.setattr(runner, 'resume_parent', lambda root: restored.append(root))
    monkeypatch.setattr(runner, 'resume_screen', lambda root: restored.append(root))
    def benchmark(args):
        launches.append(args)
        state = json.loads((output/'state.json').read_text()); job = state['jobs'][0]
        job.update(status='done', outcome={'success': True}); put(output/'state.json', state)
    monkeypatch.setattr(runner, 'run_probe', benchmark)
    runner.run(parent, screen, workspace, output)
    assert len(launches) == 1
    assert sum(c[-1] == 'osp-test-cem.service' for c in commands) == int(cem_active)
    assert sum(c[-1] == 'osp-test-screen.service' for c in commands) == 1
    assert restored[-2:] == [parent, screen]
    assert json.loads((output/'state.json').read_text())['velocity_profile_verified']


def test_pending_native_shell_is_resolved_before_a_slot_can_be_reused(tmp_path, monkeypatch):
    root = tmp_path/'root'; put(root/'state.json', {'jobs': [dict(unit='osp-old-native.service', output='/old/output')]})
    settled = [False]; observations = []
    monkeypatch.setattr(runner, 'service_active', lambda unit: True)
    monkeypatch.setattr(runner, 'native_pid', lambda unit, output: 42 if settled[0] else None)
    def wait(seconds):
        observations.append(seconds); settled[0] = True
    monkeypatch.setattr(runner.time, 'sleep', wait)
    runner.settle_claims(root)
    assert observations == [.25]
    assert settled[0]
