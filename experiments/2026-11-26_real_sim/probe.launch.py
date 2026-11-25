"""Native exam world and robot sensors, with no mission or motion command source."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '185' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This sensor probe requires isolated simulation domain185')
    fixture = Path(os.environ['REAL_SIM_FIXTURE'])
    package = Path(share('osp_mission'))
    cfg = yaml.safe_load((fixture/'scene.yaml').read_text())
    def include(path, **args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=args.items())
    resources = ':'.join(str(Path(share(p)).parent) for p in ('turtlebot4_description', 'irobot_create_description'))
    world = include(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py',
        gz_args=f'{fixture}/world/osp_studio.sdf -r -v 2 -s --headless-rendering')
    spawn = include(package/'launch/spawn.launch.py', world='osp_studio', model='standard',
        namespace='', x=str(cfg['start'][0]), y=str(cfg['start'][1]), yaw=str(cfg['start'][2]),
        camera_width='640', start_docked='false', rviz='false')
    return LaunchDescription([DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources),
        SetEnvironmentVariable('GZ_PARTITION', 'real_sim_probe_'+str(os.getpid())), world,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        TimerAction(period=3., actions=[spawn])])
