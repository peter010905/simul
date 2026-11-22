"""Install a four-direction wall coverage simulation candidate separately."""
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
    'visual_survey.py': '96b0432d42059fa5c9ad99d41e896ab0099a955e6fc974d6ec6b1eb8870d5ffd',
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
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_balanced_glance/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected Balanced Glance source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.4-four-wall-glance',
                        parent_sha256=digest(stage / 'balanced_glance_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'four_wall_glance_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
