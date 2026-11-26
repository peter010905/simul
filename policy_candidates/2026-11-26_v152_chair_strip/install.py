"""Fork RGB Legs with operator-marked chair strips for box fitting and anonymous hints."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'box_views.py': (
        ('from osp_mission import _r32_navigation_core as r32\n',
         'from osp_mission import _r32_navigation_core as r32\n'
         'from osp_mission.chair_strip_fit import box_fit_cells\n'),
        ('    r32.surface_samples(local, groups)\n',
         '    fit_grid = r32.Grid(box_fit_cells(local), local.resolution, local.origin)\n'
         '    r32.surface_samples(fit_grid, groups)\n'),
    ),
    'hybrid.py': (
        ('*, white_filter=None):', '*, white_filter=None, excluded_zone=None):'),
        ("            if r.get('white_leg') and (white_filter is None or not white_filter(point)):\n",
         "            if excluded_zone is not None and excluded_zone(point):\n"
         "                continue\n"
         "            if r.get('white_leg') and (white_filter is None or not white_filter(point)):\n"),
    ),
    'phased_hybrid.py': (
        ('clock,veto,white_filter=white_filter)\n',
         'clock,veto,white_filter=white_filter,\n'
         '                                 excluded_zone=self.schedule.glass_wall.in_zone)\n'),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'white_legs_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.33-white-legs':
        raise ValueError('Expected RGB Legs parent')
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
        helper = 'chair_strip_fit.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'chair_strip_manifest.json').write_text(json.dumps({
            'version': '1.5.2.34-chair-strip',
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
