"""Identical Gazebo world, robot and sensor bridges for each isolated trial."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(path, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())


def generate_launch_description():
    root = Path(os.environ['BENCH_ROOT'])
    trial = Path(os.environ['BENCH_RUN'])
    flow = Path(share('osp_mission'))
    native = os.environ['BENCH_POLICY'] == 'r32'
    resources = ':'.join(str(Path(share(p)).parent) for p in ('turtlebot4_description', 'irobot_create_description'))
    sim = include(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py',
                  gz_args=f"{root}/world/osp_studio.sdf -r -v 2 -s --headless-rendering")
    spawn = include(flow/'launch/spawn.launch.py', world='osp_studio', model='standard', namespace='',
                    x='3.24', y='2.40', yaw='0', camera_width='640', start_docked='false', rviz='false')
    if native:
        mission = include(Path(share('osp_project'))/'resources/mission.launch.py', config=str(root/'r32-sim.yaml'),
                          output_directory=str(trial/'native'), use_sim_time='true', bringup='true', rviz='false')
    else:
        repo = Path.home()/'simul'
        mission = include(flow/'launch/robot.launch.py', startup_guard='true', use_sim_time='true',
                          require_drive_controller='true', undock='false', rviz='false', planner='edge',
                          duration='120', navigation_flow_enabled='true', allow_mid_route_wall_inspection='false',
                          planner_start_ignore_radius_m='0.30', depth_topic='unused_depth',
                          policy_file=str(repo/'policy_releases/2026-11-13_face_intent_v2/policy.json'),
                          nav2_params=str(repo/'robot_profiles/wired_flow_v3/nav2_hardware_profile.yaml'),
                          output_dir=str(trial/'native'))
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""), DeclareLaunchArgument("use_sim_time", default_value="true"),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'), SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        SetEnvironmentVariable('GZ_PARTITION', 'pair_'+str(os.getpid())), sim,
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        TimerAction(period=3., actions=[spawn]), TimerAction(period=6., actions=[mission])])
