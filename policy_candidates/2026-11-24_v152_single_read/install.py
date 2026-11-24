"""Stage a separate 1.5.2.30 workspace from an installed or unbuilt .29 source tree."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

PARENT = {
    'detector.py': '67b6e6f482538b7be738fcfab2c67c00d95bb8b82432c49fab771a0bbf531dfa',
    'landmark_identity.py': 'e9b2828304d8d5afb2c7a2cb200fab55dba10f070dcdad19e4a8ef8a602c10ac',
    'marker_approach.py': '8797137b6d403dfeaeff7b18277d54a281ecd3898be80cbbfea5fd461cfc1deb',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    package = Path('src/osp_mission/osp_mission')
    manifest = parent / 'final_bearing_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.29-final-bearing':
        raise ValueError('Expected the Final Bearing parent')
    if any(digest(parent/package/name) != sha for name, sha in PARENT.items()):
        raise ValueError('Parent sensor or completion logic differs from .29')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent/name, stage/name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage/path.name)
        patch = Path(__file__).resolve().with_name('single_read.patch')
        subprocess.run(['patch', '-F', '0', '-p1', '-i', str(patch)], cwd=stage/package, check=True)
        info = dict(version='1.5.2.30-single-read', parent_sha256=digest(manifest),
                    sources={name: digest(stage/package/name) for name in PARENT})
        (stage/'single_read_manifest.json').write_text(json.dumps(info, indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent.resolve(), args.destination.resolve())
