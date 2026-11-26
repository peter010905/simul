"""Stage the measured wall entry repair on the frozen .46 runtime."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
PACKAGE = Path('src/osp_mission/osp_mission')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'wall_lap_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.46-wall-lap':
        raise ValueError('Expected the Wall Lap .46 workspace')
    for name, expected in source['sources'].items():
        if digest(parent / PACKAGE / name) != expected:
            raise ValueError(f'Parent source changed: {name}')
    import yaml
    laser = yaml.safe_load((parent / 'src/osp_mission/config/slam.yaml').read_text())
    if laser['slam_toolbox']['ros__parameters']['max_laser_range'] != 5.:
        raise ValueError('Expected the 5m parent')
    touched = json.loads((HERE / 'parent_sources.json').read_text())
    for entries in (touched,):
        for name, expected in entries.items():
            if digest(parent / PACKAGE / name) != expected:
                raise ValueError(f'Parent source changed: {name}')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        stage = Path(directory) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage / path.name)
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p1', '-i',
                        str(HERE / 'policy.patch')], cwd=stage, check=True)
        for name in touched:
            path = stage / PACKAGE / name
            compile(path.read_text(), str(path), 'exec')
        for name in ('wall_lap.py',):
            shutil.copyfile(HERE / name, stage / PACKAGE / name)
        (stage / 'wall_entry_manifest.json').write_text(json.dumps({
            'version': '1.5.2.47-wall-entry', 'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in (*touched,'wall_lap.py')},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
