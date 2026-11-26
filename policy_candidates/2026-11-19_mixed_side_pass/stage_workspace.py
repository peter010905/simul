"""Build an independent ROS workspace without modifying the corner candidate."""
import argparse
from pathlib import Path
import shutil


def stage(base, candidate, destination):
    target = destination / 'src/osp_mission'
    if target.exists():
        raise FileExistsError(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(base / 'src/osp_mission', target)
    for name in ('mission.py', 'side_route.py'):
        shutil.copy2(candidate / name, target / 'osp_mission' / name)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('base', type=Path)
    p.add_argument('candidate', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    stage(a.base, a.candidate, a.destination)
