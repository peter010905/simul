"""Prepare a TF timestamp-only candidate without changing frozen experiments."""
import argparse
from pathlib import Path
import shutil

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    args = parser.parse_args()
    if args.destination.exists():
        raise SystemExit('Use a new candidate path.')
    package = args.destination/'src/osp_mission'
    shutil.copytree(args.source/'src/osp_mission', package,
                    ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    path = package/'config/slam.yaml'
    parameters = yaml.safe_load(path.read_text())
    parameters['slam_toolbox']['ros__parameters']['restamp_tf'] = True
    path.write_text(yaml.safe_dump(parameters))
    shutil.copy2(Path(__file__).with_name('test_tf_restamp_runtime.py'),
                 package/'test/test_tf_restamp.py')
    print(f'TF restamp-only candidate prepared: {args.destination}')


if __name__ == '__main__':
    main()
