"""Stage a corner-first candidate from the frozen thorough Mixed package."""
import argparse
from pathlib import Path
import shutil


def stage(source, destination):
    package = destination / 'osp_mission'
    package.mkdir(parents=True, exist_ok=False)
    for path in source.glob('*.py'):
        if path.name != 'hybrid.py':
            (package / path.name).symlink_to(path.resolve())
    hybrid = (source / 'hybrid.py').read_text()
    old = 'from osp_mission.thorough_box import ThoroughBox'
    new = 'from osp_mission.angular_box import AngularBox'
    if hybrid.count(old) != 1 or hybrid.count('self.box_views = ThoroughBox()') != 1:
        raise ValueError('Expected frozen thorough Mixed source')
    hybrid = hybrid.replace(old, new).replace('self.box_views = ThoroughBox()',
                                              'self.box_views = AngularBox()')
    (package / 'hybrid.py').write_text(hybrid)
    shutil.copy2(Path(__file__).with_name('angular_box.py'), package / 'angular_box.py')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    stage(a.source, a.destination)
