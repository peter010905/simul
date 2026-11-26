"""Fork Chair Strip with automatic SLAM detection of chair rows."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'box_views.py': (
        ('from osp_mission.chair_strip_fit import box_fit_cells\n',
         'from osp_mission.auto_chair_strip import automatic_box_fit_cells\n'),
        ('    cells = grid.cells[top:r+span+1, left:c+span+1]\n',
         '    cells = automatic_box_fit_cells(grid)[top:r+span+1, left:c+span+1]\n'),
        ('    fit_grid = r32.Grid(box_fit_cells(local), local.resolution, local.origin)\n'
         '    r32.surface_samples(fit_grid, groups)\n',
         '    r32.surface_samples(local, groups)\n'),
    ),
    'phased_hybrid.py': (
        ('from osp_mission.center_wall import CenterWall\n',
         'from osp_mission.center_wall import CenterWall\n'
         'from osp_mission.auto_chair_strip import ChairCueFilter\n'),
        ('        def white_filter(point):\n',
         '        chair_strip = ChairCueFilter(grid)\n\n'
         '        def white_filter(point):\n'),
        ('                    and not self.schedule.glass_wall.includes(point))\n',
         '                    and not chair_strip.includes(point))\n'),
        (',\n                                 excluded_zone=self.schedule.glass_wall.in_zone)', ')'),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'chair_strip_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.34-chair-strip':
        raise ValueError('Expected Chair Strip parent')
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
        helper = 'auto_chair_strip.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'auto_chair_strip_manifest.json').write_text(json.dumps({
            'version': '1.5.2.35-auto-chair-strip',
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
