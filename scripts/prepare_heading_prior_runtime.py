"""Stage a SLAM heading-prior ablation without touching a frozen runtime."""
import argparse
from pathlib import Path
import shutil

import yaml


def prepare(source,destination):
    if destination.exists():raise FileExistsError('Create a new immutable runtime')
    original=yaml.safe_load((source/'src/osp_mission/config/slam.yaml').read_text())
    values=original['slam_toolbox']['ros__parameters']
    if values['angle_variance_penalty']!=1. or values['minimum_angle_penalty']!=.9:
        raise ValueError('Require preserved default SLAM heading prior for this ablation')
    package=destination/'src/osp_mission'
    shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    values.update(angle_variance_penalty=.05,minimum_angle_penalty=.5)
    (package/'config/slam.yaml').write_text(yaml.safe_dump(original,sort_keys=False))
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['source','destination']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();print(prepare(args.source.resolve(),args.destination.resolve()))
