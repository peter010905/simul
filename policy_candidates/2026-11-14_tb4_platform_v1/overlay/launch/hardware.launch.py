"""TB4 Standard bringup (optional) plus the same mission stack as Gazebo."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable


def setup(context):
    v=context.launch_configurations;pkg=Path(share('osp_mission'));actions=[]
    if v['bringup']=='true':
        actions.extend([SetEnvironmentVariable('ROBOT_NAMESPACE',v['namespace']),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('turtlebot4_bringup'))/'launch/standard.launch.py')),
                launch_arguments={'create3_param_file':str(pkg/'config/tb4_republisher.yaml')}.items())])
    shared={key:v[key] for key in ('namespace','output_dir','duration','rviz','slam_mode','session','posegraph','resume_pose','undock','policy_file')}
    shared.update(use_sim_time='false',require_drive_controller='false',planner='edge')
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/robot.launch.py')),
        launch_arguments=shared.items()))
    return actions


def generate_launch_description():
    defaults=dict(bringup='false',namespace=EnvironmentVariable('ROBOT_NAMESPACE',default_value=''),
        output_dir=str(Path.home()/'simul/results/tb4'),duration='600.0',rviz='true',slam_mode='async',
        session='new',posegraph='',resume_pose='0,0,0',undock='false',policy_file='')
    return LaunchDescription([DeclareLaunchArgument(key,default_value=value) for key,value in defaults.items()]+[OpaqueFunction(function=setup)])
