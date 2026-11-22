"""Shared SLAM, perception and mission launch for real hardware or simulation."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, RegisterEventHandler, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.event_handlers import OnProcessExit
from osp_mission.transport_config import image_environment
from osp_mission.body_scan_filter import route_costmap_scans


def setup(context):
    def value(k): return context.launch_configurations[k]
    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'
    raw_scan=value('scan_topic')
    filtering=not sim and value('body_scan_filter')=='true'
    planning_scan=('/'+'/'.join(p for p in (value('namespace').strip('/'),'scan_self_filtered') if p)) if filtering else raw_scan
    filter_actions=([Node(package='osp_mission',executable='body_scan_filter',output='screen',parameters=[{'input_topic':raw_scan,'output_topic':planning_scan,'base_frame':value('base_frame')}])] if filtering else [])
    guarded=sim and value('startup_guard')=='true'
    startup_namespace=value('namespace')
    config=yaml.safe_load((pkg/'config/slam.yaml').read_text())
    p=config['slam_toolbox']['ros__parameters']
    p.update(scan_topic=planning_scan,base_frame=value('base_frame'),resolution=float(value('resolution')),
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

    nav2_params=value('nav2_params') or str(pkg/'config/nav2.yaml')
    flow_enabled = value('navigation_flow_enabled') == 'true'
    if flow_enabled or filtering:
        nav_config = yaml.safe_load(Path(nav2_params).read_text())
        if filtering:
            nav_config = route_costmap_scans(nav_config, raw_scan, planning_scan)
        if flow_enabled:
            from osp_mission.transit_control import configure_controller
            vendor = Path(share('turtlebot4_navigation'))/'config/nav2.yaml'
            configure_controller(nav_config, yaml.safe_load(vendor.read_text()))
        if value('forward_045') == 'true':
            follower = nav_config['controller_server']['ros__parameters']['FollowPath']
            if follower['plugin'] != 'nav2_mppi_controller::MPPIController':
                raise ValueError('The 0.45 m/s profile requires the Mixed MPPI controller')
            follower['vx_max'] = 0.45
            nav_config['velocity_smoother']['ros__parameters']['max_velocity'][0] = 0.45
            nav_dest = Path(value('output_dir')) / 'nav2_forward_045.yaml'
            nav_dest.parent.mkdir(parents=True, exist_ok=True)
        else:
            nav_dest=Path(tempfile.mkdtemp(prefix='osp_body_scan_'))/'nav2.yaml'
        nav_dest.write_text(yaml.safe_dump(nav_config));nav2_params=str(nav_dest)
    navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('turtlebot4_navigation'))/'launch/nav2.launch.py')),
        launch_arguments={'use_sim_time':value('use_sim_time'),'params_file':nav2_params}.items())
    slam_action=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('slam_toolbox'))/'launch'/launch)),
        launch_arguments={'use_sim_time':value('use_sim_time'),'slam_params_file':str(dest),'autostart':'false' if guarded else 'true'}.items())
    if sim:
        # Scope this ablation to the SLAM process. RGB/depth and perception
        # keep their original large-image transport settings.
        slam_action=GroupAction([SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS','UDPv4'),slam_action])
    actions=[PushRosNamespace(value('namespace')),SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),
        *filter_actions,slam_action,
        Node(package='osp_mission',executable='detector',output='screen',additional_env=image_environment(pkg) if sim else {},parameters=[{'use_sim_time':sim,
            'rgb_topic':value('rgb_topic'),'depth_topic':value('depth_topic'),
            'camera_info_topic':value('camera_info_topic'),'scan_topic':planning_scan}]),
        Node(package='osp_mission',executable='mission',output='screen',parameters=[{'use_sim_time':sim,
            'duration':float(value('duration')),'output_dir':value('output_dir'),'undock':value('undock')=='true',
            'require_drive_controller':value('require_drive_controller')=='true',
            'planner_start_ignore_radius_m':float(value('planner_start_ignore_radius_m')),
            'navigation_flow_enabled':value('navigation_flow_enabled')=='true',
            'allow_mid_route_wall_inspection':value('allow_mid_route_wall_inspection')=='true',
            'base_frame':value('base_frame'),'scan_topic':planning_scan,'policy_file':value('policy_file'),'planner':value('planner')}])]

    if guarded:
        guard=Node(package='osp_mission',executable='startup_guard',output='screen',parameters=[{
            'use_sim_time':True,'output_dir':value('output_dir'),'base_frame':value('base_frame'),
            'scan_topic':planning_scan}])
        def start_navigation(event,context):
            if event.returncode!=0:
                return [LogInfo(msg='SLAM startup readiness failed; preserving diagnostics without releasing Nav2')]
            return [GroupAction([PushRosNamespace('/'+startup_namespace.strip('/')),
                SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),navigation])]
        # Register before starting the guard, including when it exits quickly.
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=guard,on_exit=start_navigation)),guard])
    else:actions.insert(4,navigation)
    if value('rviz')=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(pkg/'rviz/mission.rviz')],parameters=[{'use_sim_time':sim}]))
    return [GroupAction(actions)]


def generate_launch_description():
    defaults={'startup_guard':'false','use_sim_time':'false','namespace':'','slam_mode':'async','session':'new','posegraph':'',
        'resume_pose':'0,0,0','resolution':'0.05','base_frame':'base_link','scan_topic':'scan',
        'rgb_topic':'oakd/rgb/preview/image_raw','depth_topic':'unused_depth',
        'camera_info_topic':'oakd/rgb/preview/camera_info','duration':'600.0',
        'policy_file':'','planner':'legacy','nav2_params':'','output_dir':str(Path.home()/'simul/results'),'undock':'false','rviz':'true','require_drive_controller':'false','planner_start_ignore_radius_m':'0.0','body_scan_filter':'true','navigation_flow_enabled':'true','allow_mid_route_wall_inspection':'false','forward_045':'false'}
    choices={'slam_mode':['async','sync'],'session':['new','continue','localization']}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v,**({'choices':choices[k]} if k in choices else {})) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
