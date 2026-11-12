import json

import run_startup_seed_regression as runner


def terminal_state(tmp_path):
    episode=tmp_path/'case_0/episodes/123';episode.mkdir(parents=True)
    state=dict(status='running',jobs=[dict(output=str(tmp_path/'case_0'),
        scenario=dict(seed=123),status='done',outcome=dict(success=True,reason='all_targets_found'))])
    (tmp_path/'state.json').write_text(json.dumps(state))
    return episode


def test_unobserved_startup_cannot_be_reported_as_a_verified_native_success(tmp_path,monkeypatch):
    terminal_state(tmp_path)
    monkeypatch.setattr(runner,'run_reserved',lambda *args:None)
    runner.run(tmp_path,tmp_path,tmp_path)
    state=json.loads((tmp_path/'state.json').read_text())
    assert not state['startup_guard_verified']
    assert not state['jobs'][0]['outcome']['success']
    assert state['jobs'][0]['raw_outcome_before_startup_guard_check']['success']


def test_ready_startup_preserves_actual_mission_failure_instead_of_promoting_it(tmp_path,monkeypatch):
    episode=terminal_state(tmp_path)
    state=json.loads((tmp_path/'state.json').read_text())
    state['jobs'][0]['outcome']=dict(success=False,reason='timeout',seconds=600.2)
    (tmp_path/'state.json').write_text(json.dumps(state))
    (episode/'startup_guard.json').write_text(json.dumps(dict(success=True,observed_slam_state=3)))
    monkeypatch.setattr(runner,'run_reserved',lambda *args:None)
    runner.run(tmp_path,tmp_path,tmp_path)
    final=json.loads((tmp_path/'state.json').read_text())
    assert final['startup_guard_verified']
    assert final['jobs'][0]['outcome']==state['jobs'][0]['outcome']
