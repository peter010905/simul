"""Install stable print-bearing priority as a separate simulation policy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PARENT = {'center_wall.py': '0fc3e32db33bc0f0a2e0ee6aaa3fdeccf1a4618e25f6534f40ab6048882c07aa'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-23_v152_distant_wall_gain/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected parent source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.11-weak-wall-priority',
                        parent_sha256=digest(stage / 'distant_wall_gain_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'weak_wall_priority_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
