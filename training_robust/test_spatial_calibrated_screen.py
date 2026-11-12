import copy
from run_spatial_calibrated_screen import qualified_regression


def evidence():
    state = {'velocity_profile_verified': True, 'jobs': [{'status': 'done', 'outcome': {'success': True}}]}
    audit = {'physical_credit_audit': {'all_three_physical_credits_verified': True,
        'all_three_recognition_and_approach_events_logged': True,
        'rows': [{'range_witness_verified': True} for _ in range(3)]}}
    return state, audit


def test_new_space_screen_requires_actual_sensor_witnesses_for_all_three_ids():
    state, audit = evidence()
    assert qualified_regression(state, audit)
    for number in range(3):
        broken = copy.deepcopy(audit)
        broken['physical_credit_audit']['rows'][number]['range_witness_verified'] = False
        assert not qualified_regression(state, broken)
    audit['physical_credit_audit']['rows'].pop()
    assert not qualified_regression(state, audit)


def test_original_success_or_a_prior_closest_pass_cannot_bypass_completion_audit():
    state, audit = evidence()
    for field in ('all_three_physical_credits_verified', 'all_three_recognition_and_approach_events_logged'):
        broken = copy.deepcopy(audit); broken['physical_credit_audit'][field] = False
        assert not qualified_regression(state, broken)
    assert not qualified_regression(state, {})


def test_profile_startup_or_original_failure_blocks_new_space_screen():
    state, audit = evidence()
    for field, value in [('status', 'running'), ('outcome', {'success': False})]:
        broken = copy.deepcopy(state); broken['jobs'][0][field] = value
        assert not qualified_regression(broken, audit)
    state['velocity_profile_verified'] = False
    assert not qualified_regression(state, audit)
