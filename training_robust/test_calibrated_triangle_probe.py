"""Avoid competing priority launchers when the prerequisite screen decides."""
import json

import run_calibrated_triangle_probe as runner


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def make_case(tmp_path, passed):
    parent, prior, screen, workspace, output = [tmp_path/name for name in
        ('parent', 'prior', 'screen', 'workspace', 'output')]
    put(prior/'state.json', dict(status='completed_strict_approach_regression',
        velocity_profile_verified=True, jobs=[dict(status='done', outcome={'success': passed},
            output=str(prior/'case'))]))
    put(prior/'case/observer_analysis.json', {'physical_credit_audit': {
        'all_three_physical_credits_verified': passed,
        'all_three_recognition_and_approach_events_logged': passed,
        'rows': [{'range_witness_verified': passed} for _ in range(3)]}})
    put(screen/'state.json', {'status': 'running'})
    put(output/'state.json', {'status': 'running', 'driver_sha256': 'fixed'})
    return parent, prior, screen, workspace, output


def test_qualified_regression_leaves_seven_shape_dispatcher_as_only_new_launcher(tmp_path, monkeypatch):
    args = make_case(tmp_path, True); launches = []
    monkeypatch.setattr(runner, 'digest', lambda path: 'fixed')
    monkeypatch.setattr(runner, 'run_profiled_case', lambda *args: launches.append(args))
    runner.run(*args)
    assert not launches
    assert json.loads((args[-1]/'state.json').read_text())['status'] == 'not_needed_seven_shape_screen_proceeds'


def test_failed_regression_waits_until_other_dispatcher_has_stopped_without_launching(tmp_path, monkeypatch):
    args = make_case(tmp_path, False); launches = []; waits = []
    monkeypatch.setattr(runner, 'digest', lambda path: 'fixed')
    def wait(seconds):
        assert not launches
        waits.append(seconds)
        put(args[2]/'state.json', {'status': 'not_started_prerequisite_failed'})
    monkeypatch.setattr(runner.time, 'sleep', wait)
    def launch(parent, prior, workspace, output):
        assert json.loads((args[2]/'state.json').read_text())['status'] == 'not_started_prerequisite_failed'
        launches.append(output)
        state = json.loads((output/'state.json').read_text())
        assert state['seven_shape_screen_terminal']
    monkeypatch.setattr(runner, 'run_profiled_case', launch)
    runner.run(*args)
    assert len(waits) == len(launches) == 1
    assert json.loads((args[-1]/'state.json').read_text())['status'] == 'completed_calibrated_triangle_probe'


def test_unknown_dispatcher_decision_never_launches_an_extra_native(tmp_path, monkeypatch):
    args = make_case(tmp_path, False); launches = []; clock = [0.]
    monkeypatch.setattr(runner, 'digest', lambda path: 'fixed')
    monkeypatch.setattr(runner.time, 'sleep', lambda seconds: None)
    def tick():
        clock[0] += 100.
        return clock[0]
    monkeypatch.setattr(runner.time, 'monotonic', tick)
    monkeypatch.setattr(runner, 'run_profiled_case', lambda *args: launches.append(args))
    runner.run(*args)
    assert not launches
    assert json.loads((args[-1]/'state.json').read_text())['status'] == 'prerequisite_evidence_requires_attention'
