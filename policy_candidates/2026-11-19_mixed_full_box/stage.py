"""Stage full box inspection from the confirmed two-view Mixed candidate."""
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
    old = 'from osp_mission.two_views import TwoViews'
    new = 'from osp_mission.full_box import FullBox'
    if hybrid.count(old) != 1 or hybrid.count('self.box_views = TwoViews()') != 1:
        raise ValueError('Expected the confirmed two-view Mixed candidate')
    hybrid = hybrid.replace(old, new).replace('self.box_views = TwoViews()', 'self.box_views = FullBox()')
    call = 'box = self.box_views.choose(grid,grid_from_message(msg),pose,transform,owner.now(),owner.targets,veto)'
    if hybrid.count(call) != 1:
        raise ValueError('Expected one box-view selection call')
    hybrid = hybrid.replace(call, "box = (self.box_views.choose(grid,grid_from_message(msg),pose,transform,owner.now(),owner.targets,veto)\n                   if owner.p('duration')-(owner.now()-owner.started)>=45. else None)")
    (package / 'hybrid.py').write_text(hybrid)
    shutil.copy(Path(__file__).with_name('full_box.py'), package / 'full_box.py')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    stage(args.source, args.destination)
