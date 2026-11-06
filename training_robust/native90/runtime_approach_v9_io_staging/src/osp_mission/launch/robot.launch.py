"""Shared SLAM, perception and mission launch for real hardware or simulation."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from osp_mission.transport_config import image_environment


def setup(context):
    def value(k): return context.launch_configurations[k]
    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'
    config=yaml.safe_load((pkg/'config/slam.yaml').read_text())
    p=config['slam_toolbox']['ros__parameters']
    p.update(scan_topic=value('scan_topic'),base_frame=value('base_frame'),resolution=float(value('resolution')),
             scan_queue_size=1 if value('slam_mode')=='async' else 100,
             minimum_time_interval=.1)
    if value('session')!='new':
        filename=str(Path(value('posegraph')).expanduser().resolve())
        for ext in ['.posegraph','.data']:
            if not Path(filename+ext).is_file(): raise RuntimeError(f'Missing {filename+ext}; supply posegraph prefix without extension')
        p.update(map_file_name=filename,map_start_pose=[float(v) for v in value('resume_pose').split(',')])
    p['mode']='localization' if value('session')=='localization' else 'mapping'
    dest=Path(tempfile.mkdtemp(prefix='osp_slam_'))/'slam.yaml'; dest.write_text(yaml.safe_dump(config))
    launch='localization_launch.py' if value('session')=='localization' else f'online_{value("slam_mode")}_launch.py'
    actions=[PushRosNamespace(value('namespace')),SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('slam_toolbox'))/'launch'/launch)),
            launch_arguments={'use_sim_time':value('use_sim_time'),'slam_params_file':str(dest)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('turtlebot4_navigation'))/'launch/nav2.launch.py')),
            launch_arguments={'use_sim_time':value('use_sim_time'),'params_file':value('nav2_params') or str(pkg/'config/nav2.yaml')}.items()),
        Node(package='osp_mission',executable='detector',output='screen',additional_env=image_environment(pkg) if sim else {},parameters=[{'use_sim_time':sim,
            'rgb_topic':value('rgb_topic'),'depth_topic':value('depth_topic'),
            'camera_info_topic':value('camera_info_topic'),'scan_topic':value('scan_topic')}]),
        Node(package='osp_mission',executable='mission',output='screen',parameters=[{'use_sim_time':sim,
            'duration':float(value('duration')),'output_dir':value('output_dir'),'undock':value('undock')=='true',
            'require_drive_controller':value('require_drive_controller')=='true',
            'base_frame':value('base_frame'),'scan_topic':value('scan_topic'),'policy_file':value('policy_file'),'planner':value('planner')}])]
    if value('rviz')=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(pkg/'rviz/mission.rviz')],parameters=[{'use_sim_time':sim}]))
    return [GroupAction(actions)]


def generate_launch_description():
    defaults={'use_sim_time':'false','namespace':'','slam_mode':'async','session':'new','posegraph':'',
        'resume_pose':'0,0,0','resolution':'0.05','base_frame':'base_link','scan_topic':'scan',
        'rgb_topic':'oakd/rgb/preview/image_raw','depth_topic':'unused_depth',
        'camera_info_topic':'oakd/rgb/preview/camera_info','duration':'600.0',
        'policy_file':'','planner':'legacy','nav2_params':'','output_dir':str(Path.home()/'simul/results'),'undock':'false','rviz':'true','require_drive_controller':'false'}
    choices={'slam_mode':['async','sync'],'session':['new','continue','localization']}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v,**({'choices':choices[k]} if k in choices else {})) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
