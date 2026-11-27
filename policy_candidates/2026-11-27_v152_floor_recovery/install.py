"""Stage v1.5.2.67.1 from the matching .67 sources, before colcon build."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
PKG = Path('src/osp_mission/osp_mission')
PARENT = {
    PKG / 'mission.py': 'e6e161a8e5880a77f90c71eb7e9a23fb46177295dea51db96f33ff0ff2bb1253',
    PKG / 'phased_hybrid.py': '166d55bb023097b042d9881a3e62a8d8c64466298ce24bf377ce8fb08f03a97e',
    PKG.parent / 'launch/robot.launch.py': 'aefa62c0cecb1b776964bd8cc31c4c0b5f2f0847b03bde3bf6ecc3c903dcaaec',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    manifest = parent / 'wall_jogs_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.67-wall-jogs':
        raise ValueError('Expected v1.5.2.67 wall jogs')
    for source, expected in PARENT.items():
        if sha(parent/source) != expected:
            raise ValueError(f'Mismatched .67 source {source}')
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'stage'
        for directory in ('src', 'frozen-config'):
            shutil.copytree(parent/directory, stage/directory,
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for item in [*parent.glob('*.json'), parent/'nav2_override.yaml']:
            shutil.copy2(item, stage/item.name)
        subprocess.run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(HERE/'policy.patch')],
                       cwd=stage, check=True)
        added = [PKG/name for name in ('local_floor.py', 'escape_control.py')]
        for source in added:
            shutil.copy2(HERE/source.name, stage/source)
        for source in [*PARENT, *added]:
            compile((stage/source).read_bytes(), str(source), 'exec')
        data = dict(version='1.5.2.67.1-floor-recovery', parent_sha256=sha(manifest),
                    sources={str(s): sha(stage/s) for s in [*PARENT, *added]})
        (stage/'floor_recovery_manifest.json').write_text(json.dumps(data, indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
