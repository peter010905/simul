"""Build a separate Visual Survey policy that retains its active named view."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile


HERE=Path(__file__).resolve().parent
PARENT_MISSION='a58957097a1fd016f403aaab375d103e73f8e9a7c5cb21149b7a5d1fecf0e611'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def patch_stage(stage):
    package=stage/'src/osp_mission'
    mission=package/'osp_mission/mission.py'
    if digest(mission)!=PARENT_MISSION:
        raise ValueError('Expected the registered Visual Survey mission')
    subprocess.run(['patch','-F','0','-p1','-i',str(HERE/'focus.patch')],cwd=package,check=True)
    manifest=dict(version='1.5.2.1-focus-continuity',parent_mission_sha256=PARENT_MISSION,
                  mission_sha256=digest(mission),patch_sha256=digest(HERE/'focus.patch'),
                  parent_manifest_sha256=digest(stage/'visual_survey_manifest.json'))
    (stage/'focus_continuity_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


def install(release,destination):
    destination=destination.expanduser().resolve()
    if destination.exists():raise FileExistsError(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage=Path(temp)/'stage'
        parent=HERE.parent/'2026-11-22_v151_visual_survey/install.py'
        subprocess.run([sys.executable,str(parent),str(release.resolve()),str(stage)],check=True)
        patch_stage(stage)
        stage.rename(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release',type=Path)
    parser.add_argument('destination',type=Path)
    args=parser.parse_args()
    install(args.release,args.destination)
