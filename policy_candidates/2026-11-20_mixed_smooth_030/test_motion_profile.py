from copy import deepcopy
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import pytest
import yaml
from osp_mission.transit_control import configure_controller


def hardware():
    path=Path(__file__).resolve().parents[2]/'robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml'
    return yaml.safe_load(path.read_text())


def test_motion_profile_preserves_collision_and_mapping_settings():
    before=hardware();after=deepcopy(before)
    vendor=yaml.safe_load((Path(get_package_share_directory('turtlebot4_navigation'))/'config/nav2.yaml').read_text())
    configure_controller(after,vendor)
    for key in ('planner_server','collision_monitor','local_costmap','global_costmap'):
        assert after[key]==before[key]
    follow=after['controller_server']['ros__parameters']['FollowPath']
    smooth=after['velocity_smoother']['ros__parameters']
    assert follow['vx_max']==smooth['max_velocity'][0]==.30
    assert follow['ax_max']==smooth['max_accel'][0]==.35
    assert follow['ax_min']==smooth['max_decel'][0]==-.60
    assert follow['az_max']==smooth['max_accel'][2]==.70
    assert smooth['max_decel'][2]==-.90
    assert follow['wz_max']==smooth['max_velocity'][2]==.50
    assert after['behavior_server']['ros__parameters']['rotational_acc_lim']==.70
    assert after['behavior_server']['ros__parameters']['min_rotational_vel']==.10
    assert smooth['feedback']==before['velocity_smoother']['ros__parameters']['feedback']


@pytest.mark.parametrize('legacy_fast_flag',['false','true'])
def test_expanded_launch_caps_both_old_and_new_entry_flags(tmp_path,legacy_fast_flag):
    installed=Path(get_package_share_directory('osp_mission'))
    spec=importlib.util.spec_from_file_location('smooth_robot_launch',installed/'launch/robot.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    context=LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(context)
    profile=tmp_path/'hardware.yaml';profile.write_text(yaml.safe_dump(hardware()))
    context.launch_configurations.update(nav2_params=str(profile),navigation_flow_enabled='true',
        output_dir=str(tmp_path),forward_045=legacy_fast_flag,stall_recovery_enabled='true')
    group,=module.setup(context)
    include,=[a for a in group.get_sub_entities() if isinstance(a,IncludeLaunchDescription)
              and 'params_file' in dict(a.launch_arguments)]
    active=Path(dict(include.launch_arguments)['params_file'])
    assert active==tmp_path/'nav2_smooth_030.yaml'
    values=yaml.safe_load(active.read_text())
    assert values['controller_server']['ros__parameters']['FollowPath']['vx_max']==.30
    assert values['velocity_smoother']['ros__parameters']['max_velocity'][0]==.30
