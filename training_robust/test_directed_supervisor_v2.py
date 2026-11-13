import json
from pathlib import Path

import pytest

import run_directed_integrated_screen_v2 as runner


def parent(tmp_path):
    output=tmp_path/'case';episode=output/'episodes/12';episode.mkdir(parents=True)
    (episode/'live_status.json').write_text(json.dumps(dict(status=dict(state='running'))))
    (output/'policy.json').write_text('{}')
    state=dict(coordinator_unit='parent.service',jobs=[dict(number=1,status='running',unit='native.service',
        domain=124,output=str(output),scenario=dict(seed=12))])
    (tmp_path/'state.json').write_text(json.dumps(state));return state


def test_primes_every_input_gate_for_live_mission_before_waiting(tmp_path,monkeypatch):
    parent(tmp_path);calls=[]
    monkeypatch.setattr(runner.base,'service_active',lambda unit:unit=='native.service')
    monkeypatch.setattr(runner.base,'native_pid',lambda *a:77)
    monkeypatch.setattr(runner.base,'observe',lambda function,job,name:calls.append(name) or dict(complete=True))
    monkeypatch.setattr(runner.base,'floor_gate',lambda report:True)
    monkeypatch.setattr(runner.base,'configuration_gate',lambda *a:dict(passed=True,mismatches=[]))
    monkeypatch.setattr(runner,'parent_velocity_gate',lambda report:True)
    runner.prime_parent_inputs(tmp_path)
    job=json.loads((tmp_path/'state.json').read_text())['jobs'][0]
    assert len(calls)==3 and all(job[k] for k in ('floor_sensor_gate','runtime_configuration_gate','velocity_profile_gate'))
    assert job['status']=='running' and 'outcome' not in job


def test_terminal_native_is_not_given_missing_input_credit(tmp_path,monkeypatch):
    parent(tmp_path)
    monkeypatch.setattr(runner.base,'service_active',lambda unit:False)
    monkeypatch.setattr(runner.base,'native_pid',lambda *a:None)
    monkeypatch.setattr(runner.base,'benchmark_output_live',lambda output:False)
    runner.prime_parent_inputs(tmp_path)
    job=json.loads((tmp_path/'state.json').read_text())['jobs'][0]
    assert 'floor_sensor_gate' not in job


def test_never_writes_over_an_active_parent_dispatcher(tmp_path,monkeypatch):
    parent(tmp_path);before=(tmp_path/'state.json').read_bytes()
    monkeypatch.setattr(runner.base,'service_active',lambda unit:True)
    with pytest.raises(RuntimeError,match='must be stopped'):runner.prime_parent_inputs(tmp_path)
    assert (tmp_path/'state.json').read_bytes()==before


def test_transient_probe_gap_retries_same_live_native(tmp_path,monkeypatch):
    parent(tmp_path);attempts=[]
    monkeypatch.setattr(runner.base,'service_active',lambda unit:unit=='native.service')
    monkeypatch.setattr(runner.base,'native_pid',lambda *a:77)
    def probe(function,job,name):
        attempts.append(name)
        return None if len(attempts)==1 else dict(complete=True)
    monkeypatch.setattr(runner.base,'observe',probe)
    monkeypatch.setattr(runner.base,'floor_gate',lambda report:True)
    monkeypatch.setattr(runner.base,'configuration_gate',lambda *a:dict(passed=True,mismatches=[]))
    monkeypatch.setattr(runner,'parent_velocity_gate',lambda report:True)
    monkeypatch.setattr(runner.time,'sleep',lambda *a:None)
    runner.prime_parent_inputs(tmp_path)
    assert attempts[:2]==['floor_probe.json','floor_probe.json'] and len(attempts)==4
