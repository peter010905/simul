"""Create an independent v1.5.1-based workspace for simulation or replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

def install(release, destination):
    here=Path(__file__).resolve().parent
    if destination.exists():raise FileExistsError(destination)
    manifest=json.loads((release/'source_manifest.json').read_text())
    if manifest['version']!='1.5.1':raise ValueError('Expected frozen 1.5.1 release')
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for root,table in [(release/'src/osp_mission',manifest['source_sha256']),
                       (release,manifest['external_config_sha256'])]:
        if any(digest(root/p)!=h for p,h in table.items()):raise ValueError('Parent hash mismatch')
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage=Path(temp)/'stage'
        shutil.copytree(release/'src',stage/'src',ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        config=stage/'frozen-config';config.mkdir()
        for name in manifest['external_config_sha256']:shutil.copy2(release/name,config/name)
        pkg=stage/'src/osp_mission/osp_mission'
        subprocess.run(['patch','-p1','-i',str(here/'integration.patch')],cwd=pkg,check=True)
        for name in ('face_tasks.py','object_survey.py','survey_hybrid.py'):shutil.copy2(here/name,pkg/name)
        output=dict(parent='1.5.1-balanced',version='1.5.1-object-survey',source_sha256={
            name:digest(pkg/name) for name in ('center_wall.py','mission.py','phased_hybrid.py','face_tasks.py','object_survey.py','survey_hybrid.py')})
        (stage/'object_survey_manifest.json').write_text(json.dumps(output,indent=2)+'\n')
        stage.rename(destination)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('release',type=Path);p.add_argument('destination',type=Path)
    a=p.parse_args();install(a.release,a.destination)
