"""Install camera-evidence-aware 12-zone wall survey separately."""
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
    'center_wall.py': '0da872692565949e42917d2ce4de8b6e6de904b18e978a259a3212a4ef4f29ee',
    'visual_survey.py': '7076497742fd3a319dd5fd6ea8308f3a9d1f5dcb973e6b8e1c87e4a13f6b425d',
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
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_twelve_wall_views/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected Twelve Wall Views source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.7-twelve-wall-coverage',
                        parent_sha256=digest(stage / 'twelve_wall_views_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'twelve_wall_coverage_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
