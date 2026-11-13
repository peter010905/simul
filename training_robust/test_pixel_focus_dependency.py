import json

import pytest

import run_pixel_focus_regression as runner


def state(path,status):
    value=dict(coordinator_unit='preceding.service',jobs=[dict(status=status,unit='preceding-native.service',output='/preceding')])
    (path/'state.json').write_text(json.dumps(value))


def test_waits_for_old_owner_final_callback_even_after_job_is_done(tmp_path,monkeypatch):
    dependency=tmp_path/'old';dependency.mkdir();output=tmp_path/'new';output.mkdir();state(dependency,'done')
    activity=iter([True,False]);waits=[]
    monkeypatch.setattr(runner,'service_active',lambda unit:next(activity))
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:False)
    monkeypatch.setattr(runner.time,'sleep',lambda seconds:waits.append(seconds))
    runner.wait_for_dependency(dependency,output)
    assert waits==[3]


def test_live_native_handle_prevents_terminal_classification_of_unloaded_owner(tmp_path,monkeypatch):
    dependency=tmp_path/'old';dependency.mkdir();output=tmp_path/'new';output.mkdir();state(dependency,'running')
    monkeypatch.setattr(runner,'service_active',lambda unit:False)
    monkeypatch.setattr(runner,'native_pid',lambda *args:77)
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:True)
    def advance(seconds):
        state(dependency,'done')
        monkeypatch.setattr(runner,'benchmark_output_live',lambda output:False)
    monkeypatch.setattr(runner.time,'sleep',advance)
    runner.wait_for_dependency(dependency,output)


def test_terminal_uncollected_dependency_needs_collection_without_starting_new_native(tmp_path,monkeypatch):
    dependency=tmp_path/'old';dependency.mkdir();output=tmp_path/'new';output.mkdir();state(dependency,'running')
    monkeypatch.setattr(runner,'service_active',lambda unit:False)
    monkeypatch.setattr(runner,'native_pid',lambda *args:None)
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:False)
    with pytest.raises(RuntimeError,match='repair its collection'):runner.wait_for_dependency(dependency,output)


def test_transient_observation_timeout_repolls_same_dependency_instead_of_restarting(tmp_path,monkeypatch):
    dependency=tmp_path/'old';dependency.mkdir();output=tmp_path/'new';output.mkdir();state(dependency,'done')
    attempts=[]
    def active(unit):
        attempts.append(unit)
        if len(attempts)==1:raise runner.subprocess.TimeoutExpired(['systemctl'],5)
        return False
    monkeypatch.setattr(runner,'service_active',active)
    monkeypatch.setattr(runner,'benchmark_output_live',lambda output:False)
    monkeypatch.setattr(runner.time,'sleep',lambda seconds:None)
    runner.wait_for_dependency(dependency,output)
    assert attempts==['preceding.service','preceding.service']
    assert json.loads((output/'dependency_observation_gap.json').read_text())['native_not_restarted']
