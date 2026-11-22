"""Install a separate wall/object balanced-view simulation candidate."""
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
    'center_wall.py': 'f3598710cece5e8a8914fa2882ede2041206a12791d69ef9624b34d71e4aa3af',
    'visual_survey.py': '0eac528837e10e5aafd264d39feb11f70d87a357b4894b5b476d1543f579a331',
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
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_wall_spread/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected Wall Spread source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.3-balanced-glance',
                        parent_sha256=digest(stage / 'wall_spread_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'balanced_glance_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
