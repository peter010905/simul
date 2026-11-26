"""Stage the candidate on top of the existing, unmodified adaptive-face simulation workspace."""
import argparse
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory


def stage(base, destination):
    if destination.exists():
        raise FileExistsError(destination)
    folder = Path(__file__).resolve().parent
    with TemporaryDirectory(dir=destination.parent) as tmp:
        root = Path(tmp) / 'workspace'
        package = root / 'src/osp_mission'
        shutil.copytree(base / 'src/osp_mission', package,
                        ignore=shutil.ignore_patterns('*.pyc','__pycache__'))
        code = package / 'osp_mission'
        subprocess.run(['patch','--batch','--forward','--fuzz=0','-p0','-i',str(folder/'scheduler.patch')],
                       cwd=code, check=True)
        shutil.copy2(folder/'tracked_faces.py', code/'tracked_faces.py')
        root.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('adaptive_workspace', type=Path)
    parser.add_argument('new_workspace', type=Path)
    args = parser.parse_args()
    stage(args.adaptive_workspace.resolve(), args.new_workspace.resolve())
