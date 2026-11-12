"""Protect strict native scoring and slot accounting during legacy draining."""
import json
import subprocess
from pathlib import Path

import pytest
import spatial_native_curriculum_v6 as runner


@pytest.mark.parametrize('actual,owned_pid,expected', [([77], None, 2), ([77], 77, 1), ([], None, 1)])
def test_accepted_shell_and_foreign_native_each_reserve_a_slot(monkeypatch, actual, owned_pid, expected):
    monkeypatch.setattr(runner, 'native_processes', lambda: actual)
    monkeypatch.setattr(runner, 'native_pid', lambda unit, output: owned_pid)
    jobs = [dict(status='running', unit='osp-owned.service', output='/owned')]
    assert runner.occupied_slots(jobs) == expected


def test_actual_closest_pass_or_map_credit_alone_cannot_pass_strict_credit_gate():
    base = dict(all_three_physical_credits_verified=True,
        all_three_recognition_and_approach_events_logged=True,
        rows=[dict(range_witness_verified=True) for _ in range(3)])
    assert runner.strict_credit_gate(base)
    for number in range(3):
        value = json.loads(json.dumps(base)); value['rows'][number]['range_witness_verified'] = False
        assert not runner.strict_credit_gate(value)
    base['all_three_recognition_and_approach_events_logged'] = False
    assert not runner.strict_credit_gate(base)


@pytest.mark.parametrize('physical_ok,window,role,expected', [
    (True,1.0,'0',True), (False,1.0,'0',False), (True,None,'0',False), (True,None,'baseline',True)])
def test_native_success_requires_physical_proofs_and_correct_source_algorithm(tmp_path, monkeypatch, physical_ok, window, role, expected):
    directory = tmp_path/'job'; directory.mkdir()
    result = dict(success=True,targets=3,elapsed_seconds=200.,collisions=0,reason='all_targets_found',
        ground_truth_audit={'approached_within_1_5m':3},approach={'witness_window_sim_s':window})
    (directory/'report.json').write_text(json.dumps({'rows':[{'result':result}]}))
    monkeypatch.setattr(runner,'audit_episode',lambda *args:dict(success=True))
    monkeypatch.setattr(runner,'audit_credits',lambda *args:dict(
        all_three_physical_credits_verified=physical_ok,
        all_three_recognition_and_approach_events_logged=True,
        rows=[dict(range_witness_verified=physical_ok) for _ in range(3)]))
    job = dict(output=str(directory),scenario={'seed':1,'file':str(tmp_path/'scene.yaml')},role=role)
    runner.collect(job)
    assert job['status']=='done'
    assert job['outcome']['success'] is expected
    assert job['outcome']['seconds']==200.
    assert (directory/'physical_credit_audit.json').exists()


def test_all_training_roles_share_the_corrected_runtime_while_baseline_stays_frozen():
    config={'workspace':'/corrected','baseline_workspace':'/frozen_predecessor'}
    assert {runner.training_workspace(config,{},'train',str(i)) for i in range(4)}=={'/corrected'}
    assert runner.training_workspace(config,{},'pilot','baseline')=='/frozen_predecessor'


def test_probe_timeout_is_recorded_without_ending_or_restarting_live_job(tmp_path):
    job=dict(status='running',domain=124,output=str(tmp_path))
    def timeout(*args): raise subprocess.TimeoutExpired(['read_only_probe'],30)
    assert runner.observe(timeout,job,'parameters.json') is None
    assert job['status']=='running'
    assert json.loads((tmp_path/'parameters.observation_error.json').read_text())['native_not_restarted']
