"""Install a separate candidate that finishes nearby box faces before changing regions."""
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
    'center_wall.py': 'c0cd6e96dd41e00c5173f7407928e90abce9b39c99b91da897efc3769bff8697',
    'box_batch.py': 'a4c0e2d900735740c0cdbb50a50a542c8203a6d48fb84718fdfe719dc578e8ab',
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
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_wall_marginal_gain/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected parent source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.9-local-face-commit',
                        parent_sha256=digest(stage / 'wall_marginal_gain_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'local_face_commit_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
