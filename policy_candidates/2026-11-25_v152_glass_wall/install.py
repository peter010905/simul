"""Fork Start Sweep into a glass-wall-aware candidate without editing its parent."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PARENT_CENTER_SHA = '70bafd4eb1e1fa6a220101112cd81605e6cafbe9c6bb8ffb2b64be3cf63f2800'
PARENT_REGIONAL_SHA = 'd934e6bc3f1b5c33e6b9f2388107e792089f5b674e4149643bce7beb7663c1ec'
PARENT_VISUAL_SHA = 'c1fccf4d7282f791e374b59bac004717c4ec22fb561a3a83f476669cf35b236b'
CHANGES = (
    ('from osp_mission.cluster_exit import ClusterExit\n',
     'from osp_mission.cluster_exit import ClusterExit\nfrom osp_mission.glass_wall_mask import GlassWallMask\n'
     'from osp_mission.marker_cooldown import exploration_gain\n'),
    ('        self.cluster = ClusterExit()\n',
     '        self.cluster = ClusterExit()\n        self.glass_wall = GlassWallMask()\n'),
    ('        samples = surface_cells(grid, pose, self.marked_boxes, targets)\n',
     "        choices = tuple(choice for choice in choices\n"
     "                        if not self.glass_wall.includes((choice['x'], choice['y']), .8))\n"
     '        marker_positions = tuple(targets.values())\n'
     '        if marker_positions:\n'
     '            adjusted = []\n'
     '            for choice in choices:\n'
     "                factor = exploration_gain((choice['x'], choice['y']), marker_positions)\n"
     '                if factor < 1.:\n'
     '                    choice = dict(choice)\n'
     "                    choice['features'] = np.asarray(choice['features'], dtype=float).copy()\n"
     "                    choice['features'][:2] *= factor\n"
     "                    choice['marker_gain'] = factor\n"
     '                adjusted.append(choice)\n'
     '            choices = tuple(adjusted)\n'
     '        samples = surface_cells(grid, pose, self.marked_boxes, targets)\n'
     '        samples = self.glass_wall.filter_samples(samples, grid)\n'),
    ("        weak_walls = candidates['wall']\n",
     "        marker_cooled = 0\n"
     "        if marker_positions:\n"
     "            for category, views in candidates.items():\n"
     "                adjusted = []\n"
     "                for cost, goal in views:\n"
     "                    factor = exploration_gain(goal['surface_map'], marker_positions)\n"
     "                    marker_cooled += factor < 1.\n"
     "                    adjusted.append((cost + 3. * (1. - factor), dict(goal, marker_gain=factor)))\n"
     "                candidates[category] = adjusted\n"
     "        weak_walls = candidates['wall']\n"),
    ("        self.counts = {k:len(v) for k,v in candidates.items()}\n",
     "        self.counts = {k:len(v) for k,v in candidates.items()}\n"
     "        self.counts['glass_wall_skipped'] = self.glass_wall.skipped_groups\n"
     "        self.counts['marker_cooled'] = marker_cooled\n"),
    ("        ordered = sorted(candidates['interior'], key=self.batch.rank)\n",
     "        ordered = sorted(candidates['interior'], key=self.batch.rank)\n"
     "        before_chair_strip = len(ordered)\n"
     "        ordered = [item for item in ordered if not self.glass_wall.skip_generic(item[1])]\n"
     "        self.counts['glass_chair_skipped'] = before_chair_strip - len(ordered)\n"
     "        self.counts['interior'] = len(ordered)\n"),
)

REGIONAL_CHANGES = (
    ("            views.append(View(goal, key, cell, walk, weight))\n",
     "            views.append(View(goal, key, cell, walk, weight * goal.get('marker_gain', 1.)))\n"),
)

VISUAL_CHANGES = (
    ("            entries.append((min(2.,age/period)+min(2.,direction_age/period)-.085*walk,g,key))\n",
     "            priority = min(2.,age/period)+min(2.,direction_age/period)-.085*walk\n"
     "            entries.append((priority - 3. * (1. - g.get('marker_gain', 1.)),g,key))\n"),
    ("                score=age+gain/(1.+.14*goal['distance'])\n",
     "                score=age+gain/(1.+.14*goal['distance'])-3.*(1.-g.get('marker_gain',1.))\n"),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def adapt(source):
    for old, new in CHANGES:
        if source.count(old) != 1:
            raise ValueError('Unexpected Start Sweep source at: ' + old.strip())
        source = source.replace(old, new)
    compile(source, 'center_wall.py', 'exec')
    return source


def adapt_exact(source, changes, filename):
    for old, new in changes:
        if source.count(old) != 1:
            raise ValueError('Unexpected ' + filename + ' source at: ' + old.strip())
        source = source.replace(old, new)
    compile(source, filename, 'exec')
    return source


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'start_sweep_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.31-start-sweep':
        raise ValueError('Expected Start Sweep parent')
    if digest(parent / PACKAGE / 'center_wall.py') != PARENT_CENTER_SHA:
        raise ValueError('Start Sweep center_wall.py has changed')
    if digest(parent / PACKAGE / 'regional_surface.py') != PARENT_REGIONAL_SHA:
        raise ValueError('Start Sweep regional_surface.py has changed')
    if digest(parent / PACKAGE / 'visual_survey.py') != PARENT_VISUAL_SHA:
        raise ValueError('Start Sweep visual_survey.py has changed')
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage = Path(temporary) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage / path.name)
        center = stage / PACKAGE / 'center_wall.py'
        center.write_text(adapt(center.read_text()))
        regional = stage / PACKAGE / 'regional_surface.py'
        regional.write_text(adapt_exact(regional.read_text(), REGIONAL_CHANGES, regional.name))
        visual = stage / PACKAGE / 'visual_survey.py'
        visual.write_text(adapt_exact(visual.read_text(), VISUAL_CHANGES, visual.name))
        glass = stage / PACKAGE / 'glass_wall_mask.py'
        shutil.copyfile(Path(__file__).with_name('glass_wall_mask.py'), glass)
        cooldown = stage / PACKAGE / 'marker_cooldown.py'
        shutil.copyfile(Path(__file__).with_name('marker_cooldown.py'), cooldown)
        (stage / 'glass_wall_manifest.json').write_text(json.dumps({
            'version': '1.5.2.32-glass-wall',
            'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in
                        ('center_wall.py', 'regional_surface.py', 'visual_survey.py',
                         'glass_wall_mask.py', 'marker_cooldown.py')},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
