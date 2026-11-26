"""Create an isolated .49 workspace; never update a parent or board default."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile

HERE = Path(__file__).resolve().parent
PACKAGE = Path('src/osp_mission/osp_mission')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'wall_departure_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.48-wall-departure':
        raise ValueError('Expected the frozen .48 parent')
    expected = json.loads((HERE / 'parent_sources.json').read_text())
    for name, value in expected.items():
        if sha(parent / PACKAGE / name) != value:
            raise ValueError('Parent source changed: ' + name)
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage = Path(temporary) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent/name, stage/name,
                            ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        for file in parent.glob('*_manifest.json'):
            shutil.copyfile(file, stage/file.name)
        for file in (HERE/'overlay/osp_mission').glob('*.py'):
            compile(file.read_text(), str(file), 'exec')
            shutil.copyfile(file, stage/PACKAGE/file.name)
        (stage/'work_coordinator_manifest.json').write_text(json.dumps(dict(
            version='1.5.2.49-work-coordinator', parent_sha256=sha(manifest),
            sources={f.name: sha(stage/PACKAGE/f.name)
                     for f in (HERE/'overlay/osp_mission').glob('*.py')},
            physical_drive=False, board_default_changed=False), indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent',type=Path)
    p.add_argument('destination',type=Path)
    a=p.parse_args()
    install(a.parent,a.destination)
