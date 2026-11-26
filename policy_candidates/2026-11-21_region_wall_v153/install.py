"""Build a separate source tree from an existing frozen wall-transit r2 stage."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    manifest = json.loads((parent/'wall_transit_manifest.json').read_text())
    if manifest['version'] != '1.5.2-wall-transit-r2':
        raise ValueError('Expected wall transit r2')
    source = parent/'src/osp_mission'
    hashes = {**manifest['unchanged_sha256'], **{'osp_mission/'+k:v for k,v in manifest['overlay_sha256'].items()}}
    if any(digest(source/path)!=expected for path,expected in hashes.items()):
        raise ValueError('Parent source does not match manifest')
    here = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        tree = Path(tmp)/'stage'
        pkg = tree/'src/osp_mission'
        shutil.copytree(source,pkg,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        modules = pkg/'osp_mission'
        subprocess.run(['patch','-p1','-i',str(here/'integration.patch')], cwd=modules,check=True)
        shutil.copy2(here/'region_wall.py',modules)
        paths = [*hashes,'osp_mission/region_wall.py']
        result = {name:digest(pkg/name) for name in paths}
        (tree/'region_manifest.json').write_text(json.dumps(dict(version='1.5.3-region-wall',parent=manifest['version'],
            changed=[name for name in result if result[name]!=hashes.get(name)],sha256=result),indent=2)+'\n')
        tree.rename(destination)


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('parent',type=Path); p.add_argument('destination',type=Path)
    args=p.parse_args()
    install(args.parent,args.destination)
