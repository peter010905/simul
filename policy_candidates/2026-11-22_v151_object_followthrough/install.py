"""Build the follow-through experiment on the frozen adaptive-survey r2 source."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def install(release, destination):
    here = Path(__file__).resolve().parent
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    parent = here.parent/'2026-11-22_v151_adaptive_survey'
    def digest(p):
        return hashlib.sha256(p.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'workspace'
        subprocess.run([sys.executable, str(parent/'install.py'), str(release.resolve()), str(stage),
                        '--revision', 'r2'], check=True)
        module = stage/'src/osp_mission/osp_mission'
        parent_manifest = stage/'adaptive_survey_manifest.json'
        expected = json.loads((here/'parent_fingerprint.json').read_text())
        actual = json.loads(parent_manifest.read_text())
        if actual != expected:
            raise RuntimeError('Unexpected adaptive-survey r2 source or navigation profile')
        subprocess.run(['patch','-F','0','-p1','-i',str(here/'integration.patch')],cwd=module,check=True)
        shutil.copy2(here/'followthrough.py', module/'followthrough.py')
        for backup in module.glob('*.orig'):
            backup.unlink()
        differences = {p.name:digest(p) for p in sorted(module.glob('*.py'))
                       if p.name in ('followthrough.py',) or p.name in actual['changed_source_sha256'] and
                       digest(p) != actual['changed_source_sha256'][p.name]}
        (stage/'followthrough_manifest.json').write_text(json.dumps({
            'version':'1.5.1-object-followthrough', 'parent_manifest_sha256':digest(parent_manifest),
            'patch_sha256':digest(here/'integration.patch'), 'child_modules':differences
        }, indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('release',type=Path)
    p.add_argument('destination',type=Path)
    args=p.parse_args()
    install(args.release,args.destination)
