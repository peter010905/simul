"""Stage state-driven SLAM startup and deferred standard Nav2 bringup."""
import argparse
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]


def replace_once(code,old,new):
    if code.count(old)!=1:raise ValueError(f'Review changed launch anchor:{old[:90]}')
    return code.replace(old,new)


def robot(code):
    code=replace_once(code,'OpaqueFunction, GroupAction','OpaqueFunction, GroupAction, RegisterEventHandler, LogInfo')
    code=replace_once(code,'from launch_ros.actions import Node, PushRosNamespace, SetRemap',
        'from launch_ros.actions import Node, PushRosNamespace, SetRemap\nfrom launch.event_handlers import OnProcessExit')
    code=replace_once(code,"    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'",
        "    pkg=Path(share('osp_mission')); sim=value('use_sim_time')=='true'\n    guarded=sim and value('startup_guard')=='true'\n    startup_namespace=value('namespace')")
    nav="""        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('turtlebot4_navigation'))/'launch/nav2.launch.py')),
            launch_arguments={'use_sim_time':value('use_sim_time'),'params_file':value('nav2_params') or str(pkg/'config/nav2.yaml')}.items()),
"""
    code=replace_once(code,nav,'')
    code=replace_once(code,"    actions=[PushRosNamespace(value('namespace'))",'''
    navigation=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('turtlebot4_navigation'))/'launch/nav2.launch.py')),
        launch_arguments={'use_sim_time':value('use_sim_time'),'params_file':value('nav2_params') or str(pkg/'config/nav2.yaml')}.items())
    actions=[PushRosNamespace(value('namespace'))''')
    code=replace_once(code,"'slam_params_file':str(dest)}.items()),",
        "'slam_params_file':str(dest),'autostart':'false' if guarded else 'true'}.items()),")
    code=replace_once(code,"    if value('rviz')=='true':",'''
    if guarded:
        guard=Node(package='osp_mission',executable='startup_guard',output='screen',parameters=[{
            'use_sim_time':True,'output_dir':value('output_dir'),'base_frame':value('base_frame'),
            'scan_topic':value('scan_topic')}])
        def start_navigation(event,context):
            if event.returncode!=0:
                return [LogInfo(msg='SLAM startup readiness failed; preserving diagnostics without releasing Nav2')]
            return [GroupAction([PushRosNamespace('/'+startup_namespace.strip('/')),
                SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),navigation])]
        # Register before starting the guard, including when it exits quickly.
        actions.extend([RegisterEventHandler(OnProcessExit(target_action=guard,on_exit=start_navigation)),guard])
    else:actions.insert(4,navigation)
    if value('rviz')=='true':''')
    code=replace_once(code,"defaults={'use_sim_time':'false',", "defaults={'startup_guard':'false','use_sim_time':'false',")
    return code


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable runtime')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'launch/robot.launch.py';path.write_text(robot(path.read_text()))
    path=package/'launch/simulation.launch.py';path.write_text(replace_once(path.read_text(),
        "shared.update(use_sim_time='true',", "shared.update(startup_guard='true',use_sim_time='true',"))
    path=package/'setup.py';path.write_text(replace_once(path.read_text(),
        "'record_video = osp_mission.record_video:main',",
        "'record_video = osp_mission.record_video:main','startup_guard = osp_mission.startup_guard:main',"))
    for name in ['startup_state.py','startup_guard.py']:
        shutil.copy2(ROOT/'training_robust/policy_candidates'/name,package/'osp_mission'/name)
    for name in ['test_startup_state_runtime.py','test_startup_guard_ros_runtime.py']:
        shutil.copy2(ROOT/'scripts'/name,package/'test'/name)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
