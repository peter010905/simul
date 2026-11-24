"""Stage a separate 1.5.2.31 workspace from an installed or unbuilt .30 source tree."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

PARENT = {
    'center_wall.py': '4b8b7d973fd1f36c066164ef4524ce200cd8edac5ab16f084ce45b87ffab8c51',
    'mission.py': '4330ac08f8691128b5dc5c71f20cf242733b5f98631af26498673af7048d994b',
    'phased_hybrid.py': 'b0e54b47dbf215f0a52f7b9e98e6cc9ddd60c52407a0c64dfaa9cd40a37a8e17',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    package = Path('src/osp_mission/osp_mission')
    manifest = parent / 'single_read_manifest.json'
    if json.loads(manifest.read_text())['version'] != '1.5.2.30-single-read':
        raise ValueError('Expected the Single Read parent')
    if any(digest(parent/package/name) != sha for name, sha in PARENT.items()):
        raise ValueError('Unexpected parent mission or scheduling source')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent/name, stage/name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage/path.name)
        patch = Path(__file__).resolve().with_name('start_sweep.patch')
        subprocess.run(['patch', '-F', '0', '-p1', '-i', str(patch)], cwd=stage/package, check=True)
        info = dict(version='1.5.2.31-start-sweep', parent_sha256=digest(manifest),
                    sources={name: digest(stage/package/name) for name in (*PARENT, 'marked_boxes.py')})
        (stage/'start_sweep_manifest.json').write_text(json.dumps(info, indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent.resolve(), args.destination.resolve())
