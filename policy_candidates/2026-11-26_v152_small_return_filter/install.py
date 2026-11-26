"""Fork Box Face Recovery with RGB-gated inspection of tiny map returns."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'auto_chair_strip.py': (
        ('    def includes(self, point):\n',
         '    def includes(self, point):\n'
         '        return self._near(point, .10)\n\n'
         '    def band_contains(self, point):\n'
         '        return self._near(point, .20)\n\n'
         '    def _near(self, point, radius):\n'),
        ('        return chair[cell] <= .10 and chair[cell] < other[cell]\n',
         '        return chair[cell] <= radius and chair[cell] < other[cell]\n'),
    ),
    'box_views.py': (
        ('from osp_mission.auto_chair_strip import automatic_box_fit_cells\n',
         'from osp_mission.auto_chair_strip import automatic_box_fit_cells\n'
         'from osp_mission.small_return_filter import exclude_small_from_fit\n'),
        ('    cells = automatic_box_fit_cells(grid)[top:r+span+1, left:c+span+1]\n',
         '    fit = exclude_small_from_fit(grid, automatic_box_fit_cells(grid))\n'
         '    cells = fit[top:r+span+1, left:c+span+1]\n'),
    ),
    'center_wall.py': (
        ('if not self.glass_wall.includes((choice[\'x\'], choice[\'y\']), .8))\n',
         'if not self.glass_wall.includes((choice[\'x\'], choice[\'y\']), .8)\n'
         '                        and not self.chair_band_contains((choice[\'x\'], choice[\'y\'])))\n'),
        ('from osp_mission.white_legs import ThinObstacles\n',
         'from osp_mission.white_legs import ThinObstacles\n'
         'from osp_mission.small_return_filter import SmallReturns\n'),
        ('        self.thin_obstacles = ThinObstacles()\n',
         '        self.thin_obstacles = ThinObstacles()\n'
         '        self.small_returns = SmallReturns()\n        self.chair_band_contains = lambda point: False\n'),
        ('        samples = self.glass_wall.filter_samples(samples, grid)\n',
         '        samples = self.glass_wall.filter_samples(samples, grid)\n'
         '        self.small_returns.update(grid)\n'
         '        unfiltered_count = len(samples)\n'
         '        samples = {key: cells for key, cells in samples.items()\n'
         '                   if cells and not all(self.small_returns.cells[cell] for cell in cells)}\n'
         '        small_groups_skipped = unfiltered_count-len(samples)\n'),
        ("        self.counts['thin_surface_skipped'] = thin_skipped\n",
         "        self.counts['thin_surface_skipped'] = thin_skipped\n"
         "        self.counts['small_return_groups_skipped'] = small_groups_skipped\n"
         "        self.counts['small_return_cells'] = int(self.small_returns.cells.sum())\n"),
    ),
    'hybrid.py': (
        ('            if excluded_zone is not None and excluded_zone(point):\n',
         "            if not r.get('white_leg') and excluded_zone is not None and excluded_zone(point):\n"),
    ),
    'mission.py': (
        ("                if combined is not None and combined is not selected:\n",
         "                if combined is None and hybrid.band_rejected_selected:\n"
         "                    selected = None\n"
         "                if combined is not None and combined is not selected:\n"),
    ),
    'phased_hybrid.py': (
        ("        if selected and any(k in selected for k in ('approach_id','focus_id')):\n",
         "        self.band_rejected_selected = False\n"
         "        if selected and any(k in selected for k in ('approach_id','focus_id')):\n"),
        ('        goal = self.schedule.decide(grid,cost,pose,transform,viewed,\n',
         '        self.schedule.chair_band_contains = chair_strip.band_contains\n'
         '        if selected is not None and chair_strip.band_contains((selected[\'x\'], selected[\'y\'])):\n'
         '            self.band_rejected_selected = True\n'
         '            selected = None\n'
         '        goal = self.schedule.decide(grid,cost,pose,transform,viewed,\n'),
        ('            return (self.schedule.thin_obstacles.near(point)\n',
         '            return ((small_returns.matches(point) or self.schedule.thin_obstacles.near(point))\n'),
        ('        chair_strip = ChairCueFilter(grid)\n',
         '        chair_strip = ChairCueFilter(grid)\n'
         '        small_returns = self.schedule.small_returns\n'
         '        small_returns.update(grid)\n'),
        ('clock,veto,white_filter=white_filter)\n',
         'clock,veto,white_filter=white_filter,\n'
         '                                 excluded_zone=small_returns.matches)\n'),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'box_face_recovery_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.36-box-face-recovery':
        raise ValueError('Expected Box Face Recovery parent')
    for name, expected in source['sources'].items():
        if digest(parent / PACKAGE / name) != expected:
            raise ValueError(f'Parent file changed: {name}')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        stage = Path(directory) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage / path.name)
        for name, changes in PATCHES.items():
            path = stage / PACKAGE / name
            code = path.read_text()
            for old, new in changes:
                if code.count(old) != 1:
                    raise ValueError(f'Unexpected {name} source at {old.strip()}')
                code = code.replace(old, new)
            compile(code, str(path), 'exec')
            path.write_text(code)
        helper = 'small_return_filter.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'small_return_filter_manifest.json').write_text(json.dumps({
            'version': '1.5.2.37-small-return-filter',
            'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in [*PATCHES, helper]},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
