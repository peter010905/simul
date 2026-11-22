"""Install the wall marginal-gain candidate without replacing earlier versions."""
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
    'center_wall.py': '32e381eb6c497410808c93f694ae8a7d1673d9152c0b45b475e9ad37120d0d56',
    'visual_survey.py': 'fc4309ccbcbe0c154622cb9daaec5227b8a1ff43a9a78eb26f32b9cee806ed48',
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
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_twelve_wall_coverage/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected parent source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.8-wall-marginal-gain',
                        parent_sha256=digest(stage / 'twelve_wall_coverage_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'wall_marginal_gain_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
