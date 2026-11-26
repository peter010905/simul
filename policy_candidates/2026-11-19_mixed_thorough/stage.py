"""Create a thorough candidate without changing the released full-box policy."""
from pathlib import Path
import argparse
import shutil


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError(f'Unexpected source near {old!r}')
    return source.replace(old, new)


def stage(source, destination):
    package = destination / 'osp_mission'
    package.mkdir(parents=True, exist_ok=False)
    for path in source.glob('*.py'):
        if path.name != 'hybrid.py':
            (package / path.name).symlink_to(path.resolve())
    hybrid = (source / 'hybrid.py').read_text()
    hybrid = replace_once(hybrid, 'from osp_mission.full_box import FullBox',
                          'from osp_mission.thorough_box import ThoroughBox')
    hybrid = replace_once(hybrid, 'from osp_mission.hybrid_borders import borders',
                          'from osp_mission.thorough_borders import borders')
    hybrid = replace_once(hybrid, 'self.box_views = FullBox()', 'self.box_views = ThoroughBox()')
    hybrid = replace_once(hybrid, "self.tracks = [r for r in self.tracks if now-r['stamp']<=20]",
                          "self.tracks = [r for r in self.tracks if now-r['stamp']<=45]")
    hybrid = replace_once(hybrid, "if nearest is not None and np.linalg.norm(nearest['point']-point)<.16:",
                          "if nearest is not None and np.linalg.norm(nearest['point']-point)<min(.30,.14+.025*np.linalg.norm(point-observer)):")
    hybrid = replace_once(hybrid, "and 0<=now-r['stamp']<=16", "and 0<=now-r['stamp']<=30")
    # Revisit a print from the observed side even after the robot has passed it.
    hybrid = replace_once(hybrid,
        '            direction=observer-point\n            direction/=np.linalg.norm(direction)\n            robot=np.asarray(pose[:2])\n            if (robot-point)@direction<=0:\n                continue',
        '            direction=observer-point\n            length=np.linalg.norm(direction)\n            if length<.1:\n                continue\n            direction/=length\n            robot=np.asarray(pose[:2])\n            on_view_side=(robot-point)@direction>0')
    hybrid = replace_once(hybrid,
        '                stay = np.linalg.norm(robot-point)<=1.5',
        '                stay = on_view_side and np.linalg.norm(robot-point)<=1.5')
    hybrid = replace_once(hybrid,
        '                if not stay and np.linalg.norm(robot-point)-standoff<.25:',
        '                if not stay and on_view_side and np.linalg.norm(robot-point)-standoff<.25:')
    hybrid = replace_once(hybrid,
        "f.write(json.dumps(dict(t=owner.now(),kind=kind,x=goal['x'],y=goal['y']))+'\\n')",
        "f.write(json.dumps(dict(t=owner.now(),kind=kind,x=goal['x'],y=goal['y'],"
        "box_view_scope=goal.get('box_view_scope')))+'\\n')")
    (package / 'hybrid.py').write_text(hybrid)
    shutil.copy2(Path(__file__).with_name('thorough_box.py'), package / 'thorough_box.py')
    shutil.copy2(Path(__file__).with_name('thorough_borders.py'), package / 'thorough_borders.py')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    stage(a.source, a.destination)
