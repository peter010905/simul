"""Patch the acknowledgement timeout in the actual Gazebo AND robot profiles."""
import argparse
from pathlib import Path
import shutil

import yaml


def prepare(source,destination):
    if destination.exists():
        raise FileExistsError('Use a new runtime; active sources are frozen')
    source_package=source/'src/osp_mission'
    launch=(source_package/'launch/simulation.launch.py').read_text()
    anchor="if not v['nav2_params']:v['nav2_params']=str(pkg/'config/nav2_gazebo.yaml')"
    if launch.count(anchor)!=1:
        raise ValueError('Review the actual Gazebo default Nav2 parameter-file routing')
    package=destination/'src/osp_mission'
    shutil.copytree(source_package,package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    for name in ('nav2_gazebo.yaml','nav2_robot_policy.yaml'):
        path=package/'config'/name
        config=yaml.safe_load(path.read_text())
        if config['bt_navigator']['ros__parameters']['default_server_timeout']!=20:
            raise ValueError(f'Review existing timeout in {name}')
        config['bt_navigator']['ros__parameters']['default_server_timeout']=1000
        path.write_text(yaml.safe_dump(config,sort_keys=False))
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    print(prepare(args.source.resolve(),args.destination.resolve()))
