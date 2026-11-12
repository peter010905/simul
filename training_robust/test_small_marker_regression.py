import json
from types import SimpleNamespace

from run_small_marker_regression import run, resume_coordinator


def test_regression_waits_for_live_jobs_and_resumes_original_learning(tmp_path, monkeypatch):
    campaign = tmp_path/'original'
    campaign.mkdir()
    jobs = []
    for index in range(2):
        output = campaign/f'job_{index}'
        output.mkdir()
        (output/'policy.json').write_text('{"planner":"edge","parameters":[1,1,1,1,1]}')
        (output/'manifest.json').write_text(json.dumps(dict(scenarios=[dict(seed=index)])))
        jobs.append(dict(number=index, status='running', unit=f'old-{index}', output=str(output),
                         scenario=dict(seed=index)))
    (campaign/'state.json').write_text(json.dumps(dict(jobs=jobs)))
    actions = []
    old_checks = []

    def active(unit):
        if unit.startswith('old-'):
            old_checks.append(unit)
            return len(old_checks) == 1
        return False

    def collected(job):
        actions.append(('collected', job['unit']))
        job.update(status='done', outcome=dict(success=True, seconds=200, targets=3))

    def executed(command, **kwargs):
        actions.append(('command', command))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr('run_small_marker_regression.service_active', active)
    monkeypatch.setattr('run_small_marker_regression.source_hashes', lambda _: {})
    monkeypatch.setattr('run_small_marker_regression.native_processes', lambda: [])
    monkeypatch.setattr('run_small_marker_regression.collect', collected)
    monkeypatch.setattr('run_small_marker_regression.subprocess.run', executed)
    monkeypatch.setattr('run_small_marker_regression.time.sleep', lambda _: None)
    output = tmp_path/'regression'
    run(campaign, tmp_path/'runtime', output)
    assert actions[0][1][2] == 'stop'
    assert actions[-1][1][2] == 'start'
    launches = [i for i, action in enumerate(actions) if action[0] == 'command'
                and 'run_native_service.py' in ' '.join(action[1])]
    baseline_collections = [i for i, action in enumerate(actions) if action[0] == 'collected'
                            and action[1].startswith('old-')]
    assert min(launches) > max(baseline_collections)
    final = json.loads((output/'regression_state.json').read_text())
    assert final['training_samples'] is False
    assert final['status'] == 'completed_regression_requires_fresh_map_validation'
    assert all(job['status'] == 'done' for job in final['jobs'])


def test_transient_coordinator_is_recreated_if_systemd_discarded_it(tmp_path, monkeypatch):
    commands = []

    def executed(command, **kwargs):
        commands.append(command)
        if command[:3] == ['systemctl', '--user', 'start']:
            return SimpleNamespace(returncode=5, stderr='Unit not found')
        if command[:3] == ['systemctl', '--user', 'show']:
            return SimpleNamespace(returncode=0, stdout='not-found\n')
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr('run_small_marker_regression.subprocess.run', executed)
    resume_coordinator(tmp_path)
    assert commands[-1][0] == 'systemd-run'
    assert '--property=MemoryMax=512M' in commands[-1]
    assert str(tmp_path) == commands[-1][-1]
