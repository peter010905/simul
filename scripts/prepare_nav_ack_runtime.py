"""Give Nav2 action acknowledgements time to arrive without relaxing motion safety."""
import argparse
from pathlib import Path
import shutil

import yaml


def prepare(source,destination):
    if destination.exists():
        raise FileExistsError('Create a new runtime; never patch a running candidate')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,
                    ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    path=package/'config/nav2_robot_policy.yaml'
    config=yaml.safe_load(path.read_text())
    params=config['bt_navigator']['ros__parameters']
    if params['default_server_timeout']!=20:
        raise ValueError('Review the original action acknowledgement timeout')
    params['default_server_timeout']=1000  # milliseconds, not mission seconds
    path.write_text(yaml.safe_dump(config,sort_keys=False))
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    print(prepare(args.source.resolve(),args.destination.resolve()))
