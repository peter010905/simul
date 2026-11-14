from pathlib import Path
import tempfile
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from osp_mission.world import generate
from osp_mission.transport_config import image_environment


def setup(context):
    v=context.launch_configurations; pkg=Path(share('osp_mission'))
    if v['render_backend'] not in ('software','system'):
        raise ValueError('render_backend must be software or system')
    # This host's Mesa NVK path drops entire GPU-LiDAR sectors even in an
    # empty square room. Software rendering passes the 640-beam geometry
    # audit; keep the workaround local to this launch and its descendants.
    renderer=[SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1')] if v['render_backend']=='software' else []
    if not v['nav2_params']:v['nav2_params']=str(pkg/'config/nav2_gazebo.yaml')
    policy_names={'legacy':'assignment_policy.json','inspection':'inspection_policy.json','directional':'directional_policy.json','surface_endpoint':'surface_endpoint_policy.json','edge':'edge_policy.json'}
    if v['planner'] not in policy_names:raise ValueError('Unknown planner')
    if not v['policy_file']:v['policy_file']=str(pkg/'config'/policy_names[v['planner']])
    recording=v['record_video']=='true'
    world,start=generate(v['scenario'],tempfile.mkdtemp(prefix='osp_world_'),recording=recording)
    resources=':'.join([str(Path(share(p)).parent) for p in ['turtlebot4_description','irobot_create_description']])
    sim=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args':f'{world} -r -v 2 -s --headless-rendering'}.items())
    spawn=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/spawn.launch.py')),
        launch_arguments={'world':'osp_studio','model':'standard','namespace':v['namespace'],'x':str(start[0]),'y':str(start[1]),'yaw':str(start[2]),'rviz':'false','camera_width':v['camera_width'],'start_docked':v['start_docked']}.items())
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    shared={k:v[k] for k in ['namespace','slam_mode','session','posegraph','resume_pose','duration','output_dir','rviz','policy_file','nav2_params','planner']}
    shared.update(startup_guard='true',use_sim_time='true',depth_topic='oakd/rgb/preview/depth',undock=v['start_docked'],require_drive_controller='true')
    mission=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/robot.launch.py')),launch_arguments=shared.items())
    video=[]
    if recording:
        video=[Node(package='ros_gz_bridge',executable='parameter_bridge',
                additional_env=image_environment(pkg),
                arguments=['/mission/overview/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']),
            Node(package='osp_mission',executable='record_video',namespace=v['namespace'],output='screen',
                additional_env=image_environment(pkg),
                parameters=[{'use_sim_time':True,'output_dir':str(Path(v['output_dir'])/'video')}],
                remappings=[('/tf','tf'),('/tf_static','tf_static')])]
    gui=[] if v['headless']=='true' else [ExecuteProcess(cmd=['gz','sim','-g','--gui-config',str(pkg/'config/gazebo.config')],output='screen')]
    return [*renderer,SetEnvironmentVariable('GZ_PARTITION','osp_'+str(__import__('os').getpid())), SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),sim,bridge,*gui,TimerAction(period=3.,actions=[spawn]),
            TimerAction(period=6.,actions=[mission,*video])]


def generate_launch_description():
    pkg=Path(share('osp_mission'))
    defaults={'scenario':str(pkg/'config/robust_demo.yaml'),'headless':'false','rviz':'true','namespace':'',
        'slam_mode':'async','session':'new','posegraph':'','resume_pose':'0,0,0','duration':'600.0',
        'policy_file':'','planner':'edge','camera_width':'960','nav2_params':'','record_video':'false','render_backend':'software','start_docked':'false','output_dir':str(Path.home()/'simul/results')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
