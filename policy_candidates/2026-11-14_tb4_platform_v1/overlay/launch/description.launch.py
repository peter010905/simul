"""Upstream TB4 model with a single Ogre2 sensor engine for GPU LiDAR."""
from pathlib import Path
import xacro
from osp_mission.tb4_platform import configure_model, load_profile
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    v=context.launch_configurations
    doc=xacro.process_file(str(Path(share('turtlebot4_description'))/'urdf'/v['model']/'turtlebot4.urdf.xacro'),
                           mappings={'gazebo':'ignition','namespace':v['namespace']})
    pkg=Path(share('osp_mission'))
    profile=load_profile(pkg)
    if int(v['camera_width'])!=profile['camera_width']:
        raise ValueError('Camera resolution must match the shared TurtleBot4 profile')
    configure_model(doc,profile,pkg/'config/tb4_control.yaml')
    return [Node(package='robot_state_publisher',executable='robot_state_publisher',
                 parameters=[{'use_sim_time':True,'robot_description':doc.toxml()}],
                 remappings=[('/tf','tf'),('/tf_static','tf_static')]),
            Node(package='joint_state_publisher',executable='joint_state_publisher',
                 parameters=[{'use_sim_time':True,'rate':5}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('model',default_value='standard'),
        DeclareLaunchArgument('camera_width',default_value='250'),DeclareLaunchArgument('namespace',default_value=''),OpaqueFunction(function=setup)])
