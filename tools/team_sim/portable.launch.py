"""Matched camera-profile launch for frozen v1.2 and camera-aware v1.5."""
import os
import hashlib
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable,TimerAction,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(path,**kwargs):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=kwargs.items())


def generate_launch_description():
    root=Path(os.environ['BENCH_ROOT']);run=Path(os.environ['BENCH_RUN']);flow=Path(share('osp_mission'))
    expected=os.environ['CAMERA_POLICY_WORKSPACE']
    assert str(flow).startswith(str(Path(expected)/'install')), (flow,expected)
    cfg=yaml.safe_load((root/'scene.yaml').read_text());x,y,yaw=map(str,cfg['start'])
    resources=':'.join(str(Path(share(p)).parent) for p in ('turtlebot4_description','irobot_create_description'))
    simulation=include(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py',
                       gz_args=f'{root}/world/osp_studio.sdf -r -v 2 -s --headless-rendering')
    spawn=include(flow/'launch/spawn.launch.py',world='osp_studio',model='standard',namespace='',
                  x=x,y=y,yaw=yaw,camera_width='640',start_docked='false',rviz='false')
    random_exam=os.environ.get('TEAM_RANDOM_EXAM')=='1'
    detector={'rgb_topic':'/exam/detector_rgb'} if random_exam else {}
    prior = os.environ.get('TEAM_EXAM_PRIOR_EPISODE')
    mapping = {}
    if prior:
        if not random_exam:
            raise ValueError('Prior mapping requires a randomized exam with a frozen fixture')
        prior = Path(prior)
        previous = json.loads((prior/'protocol.json').read_text())
        current = json.loads((run/'protocol.json').read_text())
        initial = json.loads((prior/'scene.json').read_text())['robot_spawn']
        if (list(map(float, cfg['start'])) != [float(initial[k]) for k in ('x', 'y', 'yaw')] or
                previous['physical_drive'] or previous['seed'] != current['seed'] or
                previous['fixture_sha256'] != current['fixture_sha256'] or
                previous['effective_world_sha256'] != current['effective_world_sha256']):
            raise ValueError('Prior mapping requires the same simulated world and starting scene')
        graph, = (prior/'native').glob('*/map.posegraph')
        prefix = graph.with_suffix('')
        hashes = {suffix: hashlib.sha256(prefix.with_suffix(suffix).read_bytes()).hexdigest()
                  for suffix in ('.posegraph', '.data')}
        # Each exam starts a fresh odometry frame at the same physical spawn.
        # SLAM receives only the two serialized graph files.
        mapping = dict(session='continue', posegraph=str(prefix.resolve()), resume_pose='0,0,0')
        current['prior_slam_map'] = dict(source_run=prior.name, graph_sha256=hashes, initial_pose=[0.,0.,0.])
        (run/'protocol.json').write_text(json.dumps(current, indent=2)+'\n')
    mission=include(flow/'launch/robot.launch.py',startup_guard='true',use_sim_time='true',
        require_drive_controller='true',undock='false',rviz='false',planner='edge',
        duration=str(cfg['duration']),navigation_flow_enabled='true',allow_mid_route_wall_inspection='false' if random_exam else 'true',
        planner_start_ignore_radius_m='0.30',depth_topic='unused_depth',
        policy_file=str(Path(expected)/'frozen-config/policy.json'),
        nav2_params=str(Path(expected)/'frozen-config/nav2_hardware_profile.yaml'),output_dir=str(run/'native'),**detector,**mapping)
    return LaunchDescription([DeclareLaunchArgument('namespace',default_value=''),
        DeclareLaunchArgument('use_sim_time',default_value='true'),SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE','1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),SetEnvironmentVariable('GZ_PARTITION','camera_aware_'+str(os.getpid())),
        simulation,Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        Node(package='ros_gz_bridge',executable='parameter_bridge',
             arguments=['/world/osp_studio/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/world/osp_studio/pose/info','/_internal/sim_ground_truth_pose')]),
        TimerAction(period=3.,actions=[spawn]),TimerAction(period=6.,actions=[mission])])
