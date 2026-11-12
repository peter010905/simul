import json

from observe_spatial_campaign import observe


def test_finite_diagnostic_without_cem_generation_is_observable_without_mutation(tmp_path):
    state=dict(status='completed_startup_and_seed_regression',phase='diagnostic',jobs=[])
    original=json.dumps(state)
    (tmp_path/'state.json').write_text(original)
    observe(tmp_path)
    report=json.loads((tmp_path/'observer_progress.json').read_text())
    assert report['generation'] is None and report['phase']=='diagnostic'
    assert report['observation_errors']==[]
    assert (tmp_path/'state.json').read_text()==original
