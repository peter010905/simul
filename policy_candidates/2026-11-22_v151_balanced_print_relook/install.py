"""Stage a separate Balanced policy with the edge-print relook."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(release, destination):
    here = Path(__file__).resolve().parent
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    source_manifest = release/'source_manifest.json'
    if sha(source_manifest) != '03e7da0c370ed68425da0113a45c1049dcacf871bcc5fa940dc7f7d4ab796c1d':
        raise ValueError('Expected the frozen Balanced 1.5.1 release')
    parent = json.loads(source_manifest.read_text())
    package = release/'src/osp_mission'
    for root, files in ((package, parent['source_sha256']), (release, parent['external_config_sha256'])):
        if any(sha(root/name) != value for name, value in files.items()):
            raise ValueError('Source or navigation configuration differs from Balanced')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        staged = Path(temporary)/'stage'
        new = staged/'src/osp_mission'
        shutil.copytree(package, new, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
        config = staged/'frozen-config'
        config.mkdir()
        for name in parent['external_config_sha256']:
            shutil.copy2(release/name, config/name)
        mask = here/'integration.patch'
        subprocess.run(['patch', '-F', '0', '-p1', '-i', str(mask)], cwd=new, check=True)
        shutil.copy2(here/'edge_print.py', new/'osp_mission/edge_print.py')
        changed = {str(path.relative_to(new)): sha(path) for path in sorted(new.rglob('*.py'))
                   if sha(path) != parent['source_sha256'].get(str(path.relative_to(new)))}
        (staged/'balanced_print_manifest.json').write_text(json.dumps(dict(
            version='1.5.1-balanced-print-relook', parent_manifest=sha(source_manifest),
            integration_sha256=sha(mask), changed_source=changed), indent=2)+'\n')
        staged.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.release.resolve(), args.destination)
