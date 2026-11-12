import json

from spatial_native_curriculum import finish_phase, shape_quality


SHAPES = ('circle','triangle','trapezoid','rectangle','l_shape','u_shape','rooms')


def trial(shape, success=True, role='candidate', seconds=200):
    return dict(generation=0, phase='validation', role=role,
        scenario=dict(arena=dict(shape=shape)),
        outcome=dict(success=success, targets=3 if success else 2, approached=3 if success else 2,
                     contacts=0, seconds=seconds, infrastructure_failure=False))


def test_validation_requires_every_shape_not_just_average(tmp_path):
    jobs = [trial(shape) for shape in SHAPES for _ in range(4)]
    jobs += [trial(shape,False,'baseline',600) for shape in SHAPES for _ in range(4)]
    # 26/28 passes 90%, but hiding both failures in one shape is only 2/4.
    jobs[0]['outcome']['success'] = False
    jobs[1]['outcome']['success'] = False
    state = dict(generation=0,phase='validation',jobs=jobs,champion=[1]*5)
    finish_phase(tmp_path, {}, state)
    assert state['phase'] == 'train' and state['generation'] == 1
    assert shape_quality(jobs[:28])['circle']['successes'] == 2


def test_passed_validation_runs_spatial_final_before_pdf_holdout(tmp_path):
    jobs = [trial(shape) for shape in SHAPES for _ in range(4)]
    jobs += [trial(shape,False,'baseline',600) for shape in SHAPES for _ in range(4)]
    state = dict(generation=0,phase='validation',jobs=jobs,champion=[1]*5)
    finish_phase(tmp_path, {}, state)
    assert state['phase'] == 'spatial_holdout'
    assert (tmp_path/'qualified_candidate.json').exists()


def test_spatial_final_cannot_pass_by_skipping_one_hard_shape(tmp_path):
    jobs = [dict(trial(shape),phase='spatial_holdout') for shape in SHAPES for _ in range(6)]
    jobs[0]['outcome']['success'] = False
    jobs[1]['outcome']['success'] = False
    state = dict(generation=0,phase='spatial_holdout',jobs=jobs,champion=[1]*5)
    finish_phase(tmp_path, {}, state)
    assert state['phase'] == 'train' and state['spatial_holdout_consumed']
    assert not (tmp_path/'candidate_policy.json').exists()


def test_successful_spatial_final_still_requires_original_pdf_benchmark(tmp_path):
    jobs = [dict(trial(shape),phase='spatial_holdout') for shape in SHAPES for _ in range(6)]
    state = dict(generation=0,phase='spatial_holdout',jobs=jobs,champion=[1]*5,
                 reserved_holdout_consumed=False)
    finish_phase(tmp_path, {}, state)
    assert state['phase'] == 'holdout'
    assert state['reserved_holdout_consumed'] is False
    assert not (tmp_path/'candidate_policy.json').exists()


def test_handoff_accepts_an_already_unloaded_transient_coordinator(monkeypatch):
    from types import SimpleNamespace
    import start_spatial_prepared
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='not-found\n', returncode=0)
    monkeypatch.setattr(start_spatial_prepared.subprocess, 'run', run)
    start_spatial_prepared.stop_previous()
    assert len(calls) == 1
    assert 'show' in calls[0] and 'stop' not in calls[0]
