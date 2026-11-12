"""Stage simulation-only calibrated encoder twist input to the existing EKF."""
import argparse
from pathlib import Path
import shutil

import yaml

ROOT=Path(__file__).resolve().parents[1]


def prepare(source,destination,scale):
    if destination.exists():raise FileExistsError('Use a fresh immutable runtime')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'launch/control.launch.py';code=path.read_text();anchor="    ld.add_action(Node(package='robot_localization', executable='ekf_node',"
    if code.count(anchor)!=1:raise ValueError('Review changed simulation control launch')
    code=code.replace(anchor,"    ld.add_action(Node(package='osp_mission',executable='calibrated_wheel_twist',\n"
        f"        output='screen',parameters=[{{'use_sim_time':True,'linear_scale':{scale!r}}}]))\n"+anchor)
    path.write_text(code)
    path=package/'config/sim_ekf.yaml';config=yaml.safe_load(path.read_text());values=config['/**']['ros__parameters']
    if values['odom0']!='wheel/odom':raise ValueError('Require original encoder input')
    values['twist0']='wheel/twist_calibrated';values['twist0_config']=values.pop('odom0_config')
    values['twist0_queue_size']=values.pop('odom0_queue_size');values.pop('odom0');values.pop('odom0_differential')
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    path=package/'setup.py';code=path.read_text();anchor="'startup_guard = osp_mission.startup_guard:main',"
    if code.count(anchor)!=1:raise ValueError('Require staged SLAM startup guard')
    path.write_text(code.replace(anchor,anchor+"'calibrated_wheel_twist = osp_mission.calibrated_wheel_twist:main',"))
    shutil.copy2(ROOT/'training_robust/policy_candidates/calibrated_wheel_twist.py',package/'osp_mission/calibrated_wheel_twist.py')
    shutil.copy2(ROOT/'scripts/test_velocity_calibration_runtime.py',package/'test/test_velocity_calibration_runtime.py')
    shutil.copy2(ROOT/'scripts/test_calibrated_state_estimation_runtime.py',package/'test/test_sim_state_estimation.py')
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','destination']:parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--scale',type=float,required=True);args=parser.parse_args()
    print(prepare(args.source.resolve(),args.destination.resolve(),args.scale))
