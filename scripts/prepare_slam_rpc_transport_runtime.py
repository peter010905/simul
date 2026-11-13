"""Isolate a SLAM-only UDP transport ablation; preserve image transports."""
import argparse
from pathlib import Path
import shutil

from prepare_approach_witness_runtime import replace_once


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Use a new immutable source stage')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'launch/robot.launch.py';code=path.read_text()
    code=replace_once(code,'RegisterEventHandler, LogInfo\n','RegisterEventHandler, LogInfo, SetEnvironmentVariable\n')
    old='''    actions=[PushRosNamespace(value('namespace')),SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('slam_toolbox'))/'launch'/launch)),
            launch_arguments={'use_sim_time':value('use_sim_time'),'slam_params_file':str(dest),'autostart':'false' if guarded else 'true'}.items()),'''
    new='''    slam_action=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('slam_toolbox'))/'launch'/launch)),
        launch_arguments={'use_sim_time':value('use_sim_time'),'slam_params_file':str(dest),'autostart':'false' if guarded else 'true'}.items())
    if sim:
        # Scope this ablation to the SLAM process. RGB/depth and perception
        # keep their original large-image transport settings.
        slam_action=GroupAction([SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS','UDPv4'),slam_action])
    actions=[PushRosNamespace(value('namespace')),SetRemap('/tf','tf'),SetRemap('/tf_static','tf_static'),
        slam_action,'''
    code=replace_once(code,old,new);path.write_text(code)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source','destination'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
