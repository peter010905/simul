"""Create a new v1.6 stage from v1.5.1 without editing either frozen ancestor."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def install(source,destination):
    if destination.exists():raise FileExistsError(destination)
    here=Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage=Path(temporary)/'workspace';package=stage/'src/osp_mission'
        shutil.copytree(source/'src/osp_mission',package,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
        modules=package/'osp_mission';mission=modules/'mission.py';text=mission.read_text()
        old='        self.marker_focus=MarkerFocus()';assert text.count(old)==1
        text=text.replace(old,'        from osp_mission.local_commitment import CommittedFocus\n        self.marker_focus=CommittedFocus(self)',1)
        old='        from osp_mission.phased_hybrid import PhasedHybrid as Hybrid';assert text.count(old)==1
        text=text.replace(old,'        from osp_mission.local_commitment import CommittedHybrid as Hybrid',1)
        old='        self.hybrid=Hybrid() if self.edge_mode else None';assert text.count(old)==1
        text=text.replace(old,old+'\n        if self.hybrid is not None:\n            self.create_timer(.5,lambda:self.hybrid.poll_budget(self))',1)
        old="attempt=self.marker_focus.attempts[focus['focus_id']],";assert text.count(old)==1
        text=text.replace(old,"attempt_proposed=focus.get('proposed_attempt',self.marker_focus.attempts.get(focus['focus_id'],0)+1),phase='proposed',",1)
        mission.write_text(text);shutil.copy2(here/'local_commitment.py',modules/'local_commitment.py')
        original=source/'src/osp_mission';hashes={}
        for path in original.rglob('*'):
            if not path.is_file() or '__pycache__' in path.parts or '.pytest_cache' in path.parts:continue
            relative=path.relative_to(original)
            if str(relative)=='osp_mission/mission.py':continue
            assert path.read_bytes()==(package/relative).read_bytes(),relative
            hashes[str(relative)]=hashlib.sha256(path.read_bytes()).hexdigest()
        (stage/'local_commitment_manifest.json').write_text(json.dumps(dict(version='1.6.0',base='1.5.1',
            unchanged_sha256=hashes,changed=['osp_mission/mission.py'],added=['osp_mission/local_commitment.py']),indent=2)+'\n')
        subprocess.run(['python3','-m','compileall','-q',str(modules)],check=True)
        stage.rename(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frozen_exposure_sync',type=Path);parser.add_argument('new_workspace',type=Path)
    args=parser.parse_args();install(args.frozen_exposure_sync.resolve(),args.new_workspace.resolve())
