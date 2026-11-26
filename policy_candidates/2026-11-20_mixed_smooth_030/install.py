"""Create an independent 0.30 m/s smooth-motion workspace from v1.3."""
import argparse
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory


def install(source, destination):
    if destination.exists():
        raise FileExistsError(destination)
    here = Path(__file__).resolve().parent
    with TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'workspace'
        package = stage / 'src/osp_mission'
        shutil.copytree(source / 'src/osp_mission', package,
                        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
        subprocess.run(['patch', '--batch', '--forward', '--fuzz=0', '-p0', '-i',
                        str(here / 'policy.patch')], cwd=package, check=True)
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stall_recovery_workspace', type=Path)
    parser.add_argument('new_workspace', type=Path)
    args = parser.parse_args()
    install(args.stall_recovery_workspace.resolve(), args.new_workspace.resolve())
