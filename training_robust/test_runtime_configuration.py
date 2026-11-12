from copy import deepcopy
from pathlib import Path

from runtime_configuration import configuration_gate


def fixture():
    return dict(parameters=dict(bt_navigator=dict(default_server_timeout=1000,use_sim_time=True),
        mission=dict(policy_file='/trial/policy.json',duration=600.,planner='edge'),
        slam_toolbox=dict(restamp_tf=True,use_sim_time=True)))


def test_effective_profile_matches_actual_policy_and_deadline():
    data=fixture()
    assert configuration_gate(data,Path('/trial/policy.json'),1000)['passed']
    for node,field,value in [('mission','duration',1200.),('mission','policy_file','/old/policy.json'),
                              ('slam_toolbox','restamp_tf',False)]:
        bad=deepcopy(data);bad['parameters'][node][field]=value
        assert configuration_gate(bad,Path('/trial/policy.json'),1000)['mismatches']


def test_wrong_gazebo_yaml_route_is_detected_and_reference_20ms_is_allowed():
    data=fixture();data['parameters']['bt_navigator']['default_server_timeout']=20
    candidate=configuration_gate(data,Path('/trial/policy.json'),1000)
    assert not candidate['passed']
    assert candidate['mismatches'][0]['observed']==20
    assert configuration_gate(data,Path('/trial/policy.json'),20)['passed']


def test_observation_gap_is_separate_from_a_verified_mismatch():
    data=fixture();del data['parameters']['slam_toolbox']
    check=configuration_gate(data,Path('/trial/policy.json'),1000)
    assert not check['passed']
    assert check['missing'] and not check['mismatches']
