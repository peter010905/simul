"""Stage an odom-local rolling costmap driven by live sensors.

SLAM remains in the global planning map. RPP collision detection, inflation,
footprint, range marking/clearing and all motion limits are preserved.
"""
import argparse
from pathlib import Path
import shutil

import yaml


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Create a new immutable runtime')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    for name in ('nav2_gazebo.yaml','nav2_robot_policy.yaml'):
        path=package/'config'/name
        config=yaml.safe_load(path.read_text())
        local=config['local_costmap']['local_costmap']['ros__parameters']
        if local['global_frame']!='odom' or not local['rolling_window'] or local['plugins']!=['static_layer','voxel_layer','inflation_layer']:
            raise ValueError('Review changed rolling local costmap assumptions')
        local['plugins']=['voxel_layer','inflation_layer']
        path.write_text(yaml.safe_dump(config,sort_keys=False))
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
