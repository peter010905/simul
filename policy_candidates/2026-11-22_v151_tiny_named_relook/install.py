"""Build the optional tiny-print relook on an exact copy of the R3 workspace."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


PARENT_SHA256 = '9071751d693de94db49017ae843317b6992903d1286e6245ff86b16c4b09c75e'
CHANGED = {'detector.py', 'marker_pair.py', 'mission.py', 'near_pass_runtime.py', 'tiny_named_relook.py'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    here = Path(__file__).resolve().parent
    parent = here.parent / '2026-11-22_v151_object_followthrough'
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'workspace'
        subprocess.run([sys.executable, str(parent / 'install.py'), str(release.resolve()), str(stage)], check=True)
        if digest(stage / 'followthrough_manifest.json') != PARENT_SHA256:
            raise ValueError('The source or installation manifest of the parent changed')
        package = stage / 'src/osp_mission/osp_mission'
        before = {p.name: digest(p) for p in package.glob('*.py')}
        patch = here / 'integration.patch'
        subprocess.run(['patch', '-F', '0', '-p1', '-i', str(patch)], cwd=package, check=True)
        shutil.copy2(here / 'tiny_named_relook.py', package)
        for backup in package.glob('*.orig'):
            backup.unlink()
        after = {p.name: digest(p) for p in package.glob('*.py')}
        if set(before) - set(after) or {k for k in after if before.get(k) != after[k]} != CHANGED:
            raise ValueError('Unexpected set of source changes in tiny-print relook')
        manifest = dict(version='1.5.1-tiny-named-relook', parent_manifest_sha256=PARENT_SHA256,
                        patch_sha256=digest(patch), child_modules={k: after[k] for k in sorted(CHANGED)})
        (stage / 'tiny_named_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release, args.destination)
