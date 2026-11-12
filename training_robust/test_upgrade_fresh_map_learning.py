import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

import upgrade_fresh_map_learning as upgrade


@pytest.mark.parametrize('launch_fails', [False, True])
def test_handoff_drains_native_jobs_and_preserves_results_on_launch_failure(tmp_path, monkeypatch, launch_fails):
    original = tmp_path/'original'
    original.mkdir()
    perception = tmp_path/'perception'
    perception.mkdir()
    workspace = tmp_path/'perception_runtime'
    destination = tmp_path/'successor'
    audited = []
    for index in range(2):
        output = tmp_path/f'audited_{index}'
        output.mkdir()
        (output/'episode_audit.json').write_text(json.dumps(dict(success=True, errors=[])))
        audited.append(dict(output=str(output), outcome=dict(success=True)))
    (perception/'regression_state.json').write_text(json.dumps(dict(
        status='completed_regression_requires_fresh_map_validation', jobs=audited,
        candidate_source_hashes={'frozen': 'source'})))
    jobs = [dict(number=index, status='running' if index < 2 else 'prepared',
        unit=f'old-{index}', phase='train', generation=0, output=str(tmp_path/f'job_{index}'),
        scenario=dict(seed=index), parameters=[1.]*5) for index in range(20)]
    initial = dict(status='running', generation=0, phase='train', jobs=jobs,
                   champion=[1.]*5, mean=[0.]*5, std=[.4]*5)
    (original/'state.json').write_text(json.dumps(initial))
    flags = dict(stopped=False, native_alive=True, new_active=False, resumed=False)
    actions = []

    def active(unit):
        if unit.startswith('old-'):
            return flags['native_alive']
        if unit == upgrade.NEW_SERVICE:
            return flags['new_active']
        return False

    def executed(command, **kwargs):
        if command[:3] == ['systemctl', '--user', 'stop']:
            flags['stopped'] = True
            actions.append('stop_coordinator')
        if command[0] == 'systemd-run':
            assert flags['stopped'] and not flags['native_alive']
            actions.append('start_successor')
            if launch_fails:
                raise subprocess.CalledProcessError(1, command)
            flags['new_active'] = True
        return SimpleNamespace(returncode=0)

    def sleep(_):
        assert flags['stopped']
        flags['native_alive'] = False

    def collect(job):
        assert flags['stopped'] and not flags['native_alive']
        job.update(status='done', outcome=dict(success=True, seconds=200.))
        actions.append('collect_native')

    def prepare(*args):
        destination.mkdir()
        successor_workspace = destination/'runtime'
        successor_workspace.mkdir()
        (destination/'state.json').write_text(json.dumps(initial))
        (destination/'configuration.json').write_text(json.dumps(dict(source_hashes={})))
        return successor_workspace

    def resume(path):
        import fcntl
        # Recovery must release the original coordinator lock before starting it.
        with (original/'coordinator.lock').open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert json.loads((original/'state.json').read_text())['status'] == 'running'
        flags['resumed'] = True

    monkeypatch.setattr(upgrade, 'service_active', active)
    monkeypatch.setattr(upgrade, 'source_hashes', lambda _: {'frozen': 'source'})
    monkeypatch.setattr(upgrade, 'tf_decision', lambda *a: dict(enabled=False))
    monkeypatch.setattr(upgrade, 'prepare', prepare)
    monkeypatch.setattr(upgrade, 'collect', collect)
    monkeypatch.setattr(upgrade, 'native_processes', lambda: [1,2] if flags['native_alive'] else [])
    monkeypatch.setattr(upgrade, 'resume_coordinator', resume)
    monkeypatch.setattr(upgrade, 'plan', lambda *a: actions.append('plan_new_maps'))
    monkeypatch.setattr(upgrade, 'audit', lambda path: dict(no_reused_occupancy=True,
        rows=[dict(occupancy_sha256='old' if path == original else 'new')]))
    monkeypatch.setattr(upgrade.subprocess, 'run', executed)
    monkeypatch.setattr(upgrade.time, 'sleep', sleep)
    monkeypatch.setattr(upgrade.signal, 'signal', lambda *a: None)
    monkeypatch.setattr(sys, 'argv', ['upgrade', '--original', str(original),
        '--perception-regression', str(perception), '--perception-workspace', str(workspace),
        '--tf-regression', str(tmp_path/'tf'), '--destination', str(destination)])
    if launch_fails:
        with pytest.raises(subprocess.CalledProcessError):
            upgrade.main()
        assert flags['resumed'] is True
    else:
        upgrade.main()
        assert json.loads((original/'state.json').read_text())['status'] == 'superseded_by_fresh_map_cem_v2'
        assert flags['resumed'] is False
    final = json.loads((original/'state.json').read_text())
    assert all(job['status'] == 'done' and job['outcome']['success'] for job in final['jobs'][:2])
    assert actions.index('start_successor') > actions.index('collect_native') > actions.index('stop_coordinator')
