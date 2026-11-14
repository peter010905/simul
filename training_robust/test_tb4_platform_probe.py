from pathlib import Path
import copy
from tb4_platform_probe import EXPECTED, configuration_gate, motion_gate, transfer_gate


def report():
    values=copy.deepcopy(EXPECTED);values['mission']['policy_file']='/tmp/policy.json'
    return dict(parameters=values,platform=dict(passed=True,ground_truth_input=False),
                scan=dict(profile='none',received=100,acquisition_stamp_preserved=True,ground_truth_input=False))


def test_observed_official_profiles_and_policy_match():
    assert configuration_gate(report(),Path('/tmp/policy.json'))['passed']
    assert motion_gate(report()) and transfer_gate(report(),'none')


def test_sim_only_calibration_or_wrong_controller_is_not_accepted():
    r=report();r['parameters']['diffdrive_controller']['left_wheel_radius_multiplier']=1.070304
    assert configuration_gate(r,Path('/tmp/policy.json'))['mismatches']
    r=report();r['parameters']['controller_server']['FollowPath.plugin']='nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'
    assert not configuration_gate(r,Path('/tmp/policy.json'))['passed']


def test_missing_parameters_and_unverified_platform_fail_closed():
    r=report();del r['parameters']['slam_toolbox']['restamp_tf']
    assert configuration_gate(r,Path('/tmp/policy.json'))['missing']
    r=report();r['platform']['passed']=False
    assert not motion_gate(r) and not transfer_gate(r,'none')
    assert not transfer_gate(report(),'moderate')
