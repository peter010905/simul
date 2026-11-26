"""Stage this candidate in a new workspace, retaining its exact parent configuration."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def install(release, destination, revision='r2'):
    here = Path(__file__).resolve().parent
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    manifest = json.loads((release / 'source_manifest.json').read_text())
    if manifest['version'] != '1.5.1':
        raise ValueError('Expected frozen v1.5.1 parent')
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()
    for root, files in ((release / 'src/osp_mission', manifest['source_sha256']),
                        (release, manifest['external_config_sha256'])):
        for name, checksum in files.items():
            if digest(root / name) != checksum:
                raise ValueError(f'Frozen parent hash differs: {name}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        stage = Path(temporary) / 'stage'
        shutil.copytree(release / 'src', stage / 'src',
                        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
        cfg = stage / 'frozen-config'
        cfg.mkdir()
        for name in manifest['external_config_sha256']:
            shutil.copy2(release / name, cfg / name)
        package = stage / 'src/osp_mission/osp_mission'
        for patch in ('integration.patch', 'vision.patch'):
            subprocess.run(['patch', '-F', '0', '-p1', '-i', str(here / patch)], cwd=package, check=True)
        for module in ('survey.py', 'deferred_views.py', 'survey_distance.py'):
            shutil.copy2(here / module, package / module)
        if revision == 'r2':
            for patch in ('host_faces.patch', 'edge_watch.patch'):
                subprocess.run(['patch', '-F', '0', '-p1', '-i', str(here / patch)], cwd=package, check=True)
        # patch(1) on different hosts can create backup files when offsets shift.
        for backup in package.glob('*.orig'):
            backup.unlink()
        changed = {p.name: digest(p) for p in sorted(package.glob('*.py'))
                   if manifest['source_sha256'].get('osp_mission/' + p.name) != digest(p)}
        (stage / 'adaptive_survey_manifest.json').write_text(json.dumps({
            'version': f'1.5.1-adaptive-survey-{revision}', 'parent': '1.5.1-balanced',
            'changed_source_sha256': changed,
            'external_config_sha256': manifest['external_config_sha256']}, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--revision', choices=('r1','r2'), default='r2')
    args = parser.parse_args()
    install(args.release, args.destination, args.revision)
