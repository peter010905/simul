"""Install the 12-view wall survey as a separate simulation candidate."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PARENT = {
    'center_wall.py': '5ec5f12fd628f74e2e2d4ec09e9a707208c8a7e4356b8c86c1857f211072d18b',
    'visual_survey.py': '86275561cfa20bb6e7d1fa35f4ae2aa9a73442965b56687097db27c92ec05f5d',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_four_wall_sections/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected Four Wall Sections source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.6-twelve-wall-views',
                        parent_sha256=digest(stage / 'four_wall_sections_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'twelve_wall_views_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
