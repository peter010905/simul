from copy import deepcopy
from pathlib import Path

from runtime_configuration_v2 import configuration_gate


def profile():
    return dict(parameters={
        'bt_navigator':dict(default_server_timeout=1000,use_sim_time=True),
        'mission':dict(policy_file='/trial/policy.json',duration=600.,planner='edge'),
        'slam_toolbox':dict(restamp_tf=True,use_sim_time=True),
        'local_costmap/local_costmap':dict(plugins=['voxel_layer','inflation_layer'],
            global_frame='odom',rolling_window=True),
        'global_costmap/global_costmap':dict(plugins=['static_layer','obstacle_layer','inflation_layer'],
            global_frame='map'),
        'controller_server':{'FollowPath.use_collision_detection':True}})


def test_live_local_map_keeps_global_slam_and_collision_detection():
    assert configuration_gate(profile(),Path('/trial/policy.json'),1000)['passed']
    for node,field,value in [
        ('local_costmap/local_costmap','plugins',['static_layer','voxel_layer','inflation_layer']),
        ('global_costmap/global_costmap','plugins',['obstacle_layer','inflation_layer']),
        ('controller_server','FollowPath.use_collision_detection',False),
        ('local_costmap/local_costmap','rolling_window',False)]:
        data=deepcopy(profile());data['parameters'][node][field]=value
        assert configuration_gate(data,Path('/trial/policy.json'),1000)['mismatches']


def test_frozen_reference_can_retain_its_static_local_layer():
    data=profile();data['parameters']['local_costmap/local_costmap']['plugins'].insert(0,'static_layer')
    assert configuration_gate(data,Path('/trial/policy.json'),1000,live_local=False)['passed']
    assert not configuration_gate(data,Path('/trial/policy.json'),1000,live_local=True)['passed']


def test_missing_costmap_response_is_an_observation_gap():
    data=profile();del data['parameters']['local_costmap/local_costmap']
    check=configuration_gate(data,Path('/trial/policy.json'),1000)
    assert check['missing'] and not check['mismatches'] and not check['passed']
