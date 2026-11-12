"""Only dispatchers are suspended; restore after a verified native claim or cleanup."""
import json
import subprocess
import threading
import time

import pytest

import run_windowed_witness_regression as runner


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value))


def prepared(tmp_path):
    parent, screen, workspace, output = [tmp_path/name for name in ('parent', 'screen', 'workspace', 'output')]
    put(screen/'state.json', dict(status='running', coordinator_unit='osp-test-screen.service'))
    put(parent/'state.json', dict(status='running', coordinator_unit='osp-test-cem.service', jobs=[]))
    put(output/'state.json', dict(status='running', jobs=[dict(status='prepared',
        unit='osp-test-windowed-native.service', output=str(output/'case_0'))]))
    return parent, screen, workspace, output


def test_screen_is_not_restored_on_running_state_without_bound_native_pid(tmp_path, monkeypatch):
    args = prepared(tmp_path); valid = threading.Event(); restored = threading.Event(); calls = []; commands = []
    monkeypatch.setattr(runner.subprocess, 'run', lambda command, **kw: commands.append(command))
    monkeypatch.setattr(runner, 'resume_parent', lambda parent: None)
    monkeypatch.setattr(runner, 'native_pid', lambda unit, output: 12345 if valid.is_set() else None)
    def restore(screen):
        calls.append(valid.is_set()); restored.set()
    monkeypatch.setattr(runner, 'resume_screen', restore)
    def launch(parent, prior, workspace, output):
        state = json.loads((output/'state.json').read_text()); state['jobs'][0]['status'] = 'running'
        put(output/'state.json', state)
        time.sleep(2.2)  # Let the watcher observe accepted state without a native handle.
        assert not calls
        valid.set()
        assert restored.wait(4.)
        state['jobs'][0]['status'] = 'done'; put(output/'state.json', state)
    monkeypatch.setattr(runner, 'run_profiled_case', launch)
    runner.run(*args)
    assert calls and all(calls)
    assert commands == [['systemctl', '--user', 'stop', 'osp-test-screen.service'],
        ['systemctl', '--user', 'stop', 'osp-test-cem.service']]
    handoff = json.loads((args[-1]/'screen_handoff.json').read_text())
    assert handoff['verified_new_native_pid'] == 12345
    assert handoff['original_native_and_queued_scenes_preserved']


def test_failed_launch_restores_original_dispatcher_without_stopping_native_units(tmp_path, monkeypatch):
    args = prepared(tmp_path); commands = []; restored = []
    monkeypatch.setattr(runner.subprocess, 'run', lambda command, **kw: commands.append(command))
    monkeypatch.setattr(runner, 'resume_parent', lambda parent: None)
    monkeypatch.setattr(runner, 'resume_screen', lambda screen: restored.append(screen))
    def fail(*args): raise RuntimeError('Launch input unavailable')
    monkeypatch.setattr(runner, 'run_profiled_case', fail)
    with pytest.raises(RuntimeError, match='Launch input unavailable'): runner.run(*args)
    assert restored == [args[1]]
    assert commands == [['systemctl', '--user', 'stop', 'osp-test-screen.service'],
        ['systemctl', '--user', 'stop', 'osp-test-cem.service']]


def test_accepted_old_shell_claim_is_settled_before_new_launch_without_stopping_native(tmp_path, monkeypatch):
    args = prepared(tmp_path); launches = []; commands = []; restored = []; settled = [False]
    put(args[0]/'state.json', dict(status='running', coordinator_unit='osp-test-cem.service',
        jobs=[dict(unit='osp-old-native.service', output=str(tmp_path/'old'))]))
    monkeypatch.setattr(runner.subprocess, 'run', lambda command, **kw: commands.append(command))
    monkeypatch.setattr(runner, 'service_active', lambda unit: unit == 'osp-old-native.service')
    monkeypatch.setattr(runner, 'native_pid', lambda unit, output: 42 if settled[0] else None)
    def wait(seconds):
        assert not launches
        settled[0] = True
    monkeypatch.setattr(runner.time, 'sleep', wait)
    monkeypatch.setattr(runner, 'resume_parent', lambda root: restored.append(root))
    monkeypatch.setattr(runner, 'resume_screen', lambda root: restored.append(root))
    def launch(*args):
        assert settled[0]
        launches.append(args)
    monkeypatch.setattr(runner, 'run_profiled_case', launch)
    runner.run(*args)
    assert len(launches) == 1
    assert restored == [args[0], args[1]]
    assert all(command[-1] != 'osp-old-native.service' for command in commands)


def test_unloaded_screen_is_recreated_with_original_frozen_inputs_and_memory_limit(tmp_path, monkeypatch):
    screen = tmp_path/'screen'
    state = dict(status='running', coordinator_unit='osp-test-screen.service',
        parent_campaign='/old/cem', prerequisite='/old/prerequisite', workspace='/old/frozen_runtime')
    put(screen/'state.json', state); active = [False]; commands = []
    monkeypatch.setattr(runner, 'service_active', lambda unit: active[0])
    def execute(command, **kwargs):
        commands.append(command)
        if command[0] == 'systemd-run': active[0] = True
        return subprocess.CompletedProcess(command, 0, stdout='not-found\n', stderr='')
    monkeypatch.setattr(runner.subprocess, 'run', execute)
    runner.resume_screen(screen)
    command = commands[-1]
    assert command[0] == 'systemd-run'
    assert '--property=MemoryMax=512M' in command
    assert command[command.index('--parent')+1] == state['parent_campaign']
    assert command[command.index('--regression')+1] == state['prerequisite']
    assert command[command.index('--workspace')+1] == state['workspace']
    assert json.loads((screen/'state.json').read_text()) == state
