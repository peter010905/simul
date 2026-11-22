"""Install the wall-spread trial as a separate Focus Continuity policy."""
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
    'center_wall.py': '013cc8432c4a14aaf47b25d2672d94c5117d0351fd8c207d990e56ccf4028fac',
    'visual_survey.py': 'f062e0c0262cb8a872a1a9c99b198c0899587218ec50a55679d73496c71ec6e5',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'stage'
        parent = HERE.parent/'2026-11-22_v152_focus_continuity/install.py'
        subprocess.run([sys.executable, str(parent), str(release.resolve()), str(stage)], check=True)
        package = stage/'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package/name) != expected:
                raise ValueError(f'Unexpected Focus Continuity source: {name}')
            shutil.copy2(HERE/name, package/name)
        manifest = dict(version='1.5.2.2-wall-spread',
                        parent_sha256=digest(stage/'focus_continuity_manifest.json'),
                        sources={name:digest(HERE/name) for name in PARENT})
        (stage/'wall_spread_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('release',type=Path)
    p.add_argument('destination',type=Path)
    a=p.parse_args()
    install(a.release,a.destination)
