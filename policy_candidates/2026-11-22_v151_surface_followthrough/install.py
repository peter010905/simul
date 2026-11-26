"""Build a separate candidate from the exact tiny-print R4 stage."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release,destination):
    here=Path(__file__).resolve().parent
    destination=destination.expanduser().resolve()
    if destination.exists():raise FileExistsError(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        stage=Path(directory)/'workspace'
        parent=here.parent/'2026-11-22_v151_tiny_named_relook/install.py'
        subprocess.run([sys.executable,str(parent),str(release.resolve()),str(stage)],check=True)
        pin=sha(stage/'tiny_named_manifest.json')
        if pin!='9787a15a2a117ccb388d42752f1324d3c60bf819e4e2a3a017985c84ece8c37c':
            raise ValueError('Expected the tested R4 parent')
        pkg=stage/'src/osp_mission';patch=here/'integration.patch'
        subprocess.run(['patch','-F','0','-p1','-i',str(patch)],cwd=pkg,check=True)
        shutil.copy2(here/'enclosed_surface.py',pkg/'osp_mission/enclosed_surface.py')
        for f in pkg.rglob('*.orig'):f.unlink()
        changed=['launch/robot.launch.py',*(f'osp_mission/{n}.py' for n in ('mission','marker_approach','followthrough','enclosed_surface'))]
        manifest=dict(version='1.5.1-surface-followthrough',parent_sha256=pin,patch_sha256=sha(patch),
                      child_files={n:sha(pkg/n) for n in changed})
        (stage/'surface_followthrough_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        stage.rename(destination)


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('release',type=Path);cli.add_argument('destination',type=Path)
    args=cli.parse_args();install(args.release,args.destination)
