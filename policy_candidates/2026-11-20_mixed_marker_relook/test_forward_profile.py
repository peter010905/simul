"""Expand the installed robot launch without starting any nodes or publishing commands."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
import yaml


def final_config(launch, profile, output, fast):
    context = LaunchContext()
    for action in launch.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(nav2_params=str(profile), navigation_flow_enabled='true',
                                         output_dir=str(output), forward_045='true' if fast else 'false')
    group, = launch.setup(context)
    include, = [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)
                and 'params_file' in dict(a.launch_arguments)]
    path = Path(dict(include.launch_arguments)['params_file'])
    if fast:
        assert path == output / 'nav2_forward_045.yaml'
    return yaml.safe_load(path.read_text())


def test_final_nav2_profile_changes_only_two_forward_limits(tmp_path):
    installed = Path(get_package_share_directory('osp_mission'))
    path = installed / 'launch/robot.launch.py'
    spec = importlib.util.spec_from_file_location('marker_robot_launch', path)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    profile = Path(__file__).resolve().parents[2] / 'robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml'
    if not profile.is_file():
        raise FileNotFoundError('Run this test from the repository candidate directory')
    base = final_config(launch, profile, tmp_path, False)
    fast = final_config(launch, profile, tmp_path, True)
    follow = base['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow['vx_max'] == .25
    assert base['velocity_smoother']['ros__parameters']['max_velocity'][0] == .27
    follow['vx_max'] = .45
    base['velocity_smoother']['ros__parameters']['max_velocity'][0] = .45
    assert fast == base
