"""Build .43.1 from a frozen .43 workspace. Only new physical mapping sessions change."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'pending_marker_manifest.json'
    record = json.loads(manifest.read_text())
    if record['version'] != '1.5.2.43-pending-marker':
        raise ValueError('Expected .43 pending marker parent')
    for name, digest in record['sources'].items():
        if sha(parent / 'src/osp_mission/osp_mission' / name) != digest:
            raise ValueError(f'Unexpected parent module {name}')
    if destination.exists():
        raise FileExistsError(destination)
    launch = Path('src/osp_mission/launch/robot.launch.py')
    anchor = "    if value('session')!='new':\n"
    code = (parent / launch).read_text()
    if code.count(anchor) != 1:
        raise ValueError('Unrecognized SLAM launch source')
    change = """    if not sim and value('session') == 'new':
        # The initial spin supplies new lidar views even while odometry XY is steady.
        p['check_min_dist_and_heading_precisely'] = True
"""
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'stage'
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name, ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copy2(path, stage / path.name)
        updated = code.replace(anchor, change + anchor)
        compile(updated, str(launch), 'exec')
        (stage / launch).write_text(updated)
        (stage / 'sweep_mapping_manifest.json').write_text(json.dumps(dict(
            version='1.5.2.43.1-sweep-mapping', parent_sha256=sha(manifest),
            parent_launch=sha(parent/launch), launch=sha(stage/launch)), indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent,args.destination)
