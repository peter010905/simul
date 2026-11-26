"""Create an independent crowded-navigation workspace from Mixed v1.3.1."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory


def install(source, destination):
    if destination.exists():
        raise FileExistsError(destination)
    here = Path(__file__).resolve().parent
    expected = json.loads((here / 'synthetic_results.json').read_text())['preserved_baseline_source_sha256']
    for name,digest in expected.items():
        if hashlib.sha256((source/'src/osp_mission/osp_mission'/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Expected unchanged v1.3.1 source: '+name)
    with TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'workspace'
        package = stage / 'src/osp_mission'
        shutil.copytree(source / 'src/osp_mission', package,
                        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p0', '-i',
                        str(here / 'policy.patch')], cwd=package, check=True)
        shutil.copy2(here / 'crowded_navigation.py', package / 'osp_mission')
        (stage / 'candidate_version.json').write_text(json.dumps(dict(
            version='1.4.0',base='1.3.1',candidate='mixed_crowded_navigation',
            base_source_sha256=expected),indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('smooth030_workspace', type=Path)
    parser.add_argument('new_workspace', type=Path)
    args = parser.parse_args()
    install(args.smooth030_workspace.resolve(), args.new_workspace.resolve())
