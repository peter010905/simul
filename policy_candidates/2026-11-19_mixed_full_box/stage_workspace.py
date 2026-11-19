"""Build the existing Mixed ROS package with the full box observation candidate."""
from pathlib import Path
import argparse
import shutil


def stage(base, candidate, destination):
    source = base / 'src/osp_mission'
    target = destination / 'src/osp_mission'
    if target.exists():
        raise FileExistsError(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)
    for name in ('hybrid.py', 'mission.py', 'box_views.py', 'two_views.py', 'corner_pair.py', 'full_box.py'):
        shutil.copy2(candidate / name, target / 'osp_mission' / name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('base', type=Path)
    parser.add_argument('candidate', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    stage(args.base, args.candidate, args.destination)
