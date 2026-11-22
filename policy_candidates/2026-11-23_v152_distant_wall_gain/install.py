"""Install the distant unseen wall candidate as a separate policy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PARENT = {'visual_survey.py': '974f74e76b6c82e72fc0c1c0736e0432fcfb9d406dbba40c0fa8ca88344d8ae9'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        subprocess.run([sys.executable, str(HERE.parent / '2026-11-22_v152_local_face_commit/install.py'),
                        str(release.resolve()), str(stage)], check=True)
        package = stage / 'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package / name) != expected:
                raise ValueError(f'Unexpected parent source: {name}')
            shutil.copy2(HERE / name, package / name)
        manifest = dict(version='1.5.2.10-distant-wall-gain',
                        parent_sha256=digest(stage / 'local_face_commit_manifest.json'),
                        sources={name: digest(HERE / name) for name in PARENT})
        (stage / 'distant_wall_gain_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
