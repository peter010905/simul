"""Install exact tested v1.5.1 source onto a hash-checked fresh v1.2 copy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def install(source,destination):
    if destination.exists():raise FileExistsError(destination)
    here=Path(__file__).resolve().parent;original=source/'src/osp_mission'
    baseline=json.loads((here/'baseline_sha256.json').read_text())
    for relative,expected in baseline.items():
        assert sha(original/relative)==expected,relative
    overlays=json.loads((here/'runtime_sha256.json').read_text())
    for relative,expected in overlays.items():
        assert sha(here/'overlay'/relative)==expected,relative
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage=Path(temporary)/'workspace';package=stage/'src/osp_mission'
        shutil.copytree(original,package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        for relative in overlays:
            target=package/relative;target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(here/'overlay'/relative,target)
        for relative,expected in baseline.items():assert sha(package/relative)==expected,relative
        for relative,expected in overlays.items():assert sha(package/relative)==expected,relative
        (stage/'snapshot_install.json').write_text(json.dumps(dict(version='1.5.1',
            unchanged_base_files=len(baseline),runtime_overlay=overlays),indent=2)+'\n')
        subprocess.run(['python3','-m','compileall','-q',str(package/'osp_mission')],check=True)
        stage.rename(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_v12',type=Path);parser.add_argument('new_workspace',type=Path)
    args=parser.parse_args();install(args.frozen_v12.resolve(),args.new_workspace.resolve())
