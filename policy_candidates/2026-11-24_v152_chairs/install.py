"""Install a chair-clutter candidate without modifying the v1.5.2.31 parent."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
REPLACEMENTS = (
    (
        "return (self.anchor is not None and self.entered is not None and\n"
        "                self.stops < 4 and 0 <= elapsed - self.entered < 55.)",
        "return (self.anchor is not None and self.entered is not None and\n"
        "                self.stops < 4 and 0 <= elapsed - self.entered < 42.)",
    ),
    (
        "weight = (1.55 if key[0] == 'box' else 1.20) * (.75 if retry else 1.)",
        "weight = (1.55 if key[0] == 'box' else .65) * (.75 if retry else 1.)",
    ),
    (
        "if same_area and view.key in self.attempted:\n                continue",
        "if same_area and view.key in self.attempted:\n                continue\n"
        "            # Chair legs can produce many nearby generic surface goals.\n"
        "            # Limit those extra views, while retaining box and wall work.\n"
        "            if active and same_area and view.key[0] not in ('box', 'wall') and self.stops >= 1:\n"
        "                continue",
    ),
    (
        "repeat_cost = max(0., 1. - visit_age / 110.)",
        "repeat_cost = max(0., 1. - visit_age / (180. if view.key[0] not in ('box', 'wall') else 110.))",
    ),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def adapt(source):
    for old, new in REPLACEMENTS:
        if source.count(old) != 1:
            raise ValueError(f'Parent regional_surface.py differs from expected source: {old[:52]}')
        source = source.replace(old, new)
    compile(source, 'regional_surface.py', 'exec')
    return source


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'start_sweep_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.31-start-sweep':
        raise ValueError('Expected Mixed v1.5.2.31 Start Sweep parent')
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
        source = stage / PACKAGE / 'regional_surface.py'
        source.write_text(adapt(source.read_text()))
        (stage / 'chair_clutter_manifest.json').write_text(json.dumps({
            'version': '1.5.2.31-chair-clutter',
            'parent_sha256': digest(manifest),
            'sources': {'regional_surface.py': digest(source)},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
