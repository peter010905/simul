"""Stage the visual survey separately from its verified Balanced Print parent."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    here = Path(__file__).resolve().parent
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    proposal = here.parents[1]/'experiments/2026-11-22_random_exam/balanced_print'
    hint = proposal/'distant_hint.py'
    if digest(hint) != json.loads((proposal/'distant_hint_screen.json').read_text())['proposal_sha256']:
        raise ValueError('Distant-hint source differs from the pre-enrolled version')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as scratch:
        stage = Path(scratch)/'stage'
        parent = here.parent/'2026-11-22_v151_balanced_print_relook/install.py'
        subprocess.run([sys.executable, str(parent), str(release.resolve()), str(stage)], check=True)
        package = stage/'src/osp_mission'
        subprocess.run(['patch','-F','0','-p1','-i',str(here/'integration.patch')], cwd=package, check=True)
        inputs = [hint, here/'visual_survey.py']
        for path in inputs:
            shutil.copy2(path, package/'osp_mission'/path.name)
        manifest = dict(version='1.5.2-visual-survey', parent=digest(stage/'balanced_print_manifest.json'),
                        sources={p.name:digest(p) for p in [*inputs, here/'integration.patch']})
        (stage/'visual_survey_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release',type=Path)
    parser.add_argument('destination',type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
