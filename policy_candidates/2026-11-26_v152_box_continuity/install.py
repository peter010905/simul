"""Stage the box continuity candidate on the verified .43.1 runtime."""
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
    manifest = parent / 'sweep_mapping_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.43.1-sweep-mapping':
        raise ValueError('Expected the Safe Frontier .42 workspace')
    if digest(parent / 'src/osp_mission/launch/robot.launch.py') != source['launch']:
        raise ValueError('Parent launch changed')
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
        for name in ('box_continuity.py','box_tour.py'):
            shutil.copyfile(HERE / name, stage / PACKAGE / name)
        (stage / 'box_continuity_manifest.json').write_text(json.dumps({
            'version': '1.5.2.44-box-continuity', 'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in (*touched,'box_continuity.py','box_tour.py')},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
