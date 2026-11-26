"""Stage the candidate on top of the existing, unmodified box-batch simulation workspace."""
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
        subprocess.run(['patch','--batch','--forward','--fuzz=0','-p0','-i',str(folder/'wall_cues.patch')],
                       cwd=code, check=True)
        for module in ('small_wall_cues.py', 'wall_bearings.py'):
            shutil.copy2(folder/module, code/module)
        root.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('batch_workspace', type=Path)
    parser.add_argument('new_workspace', type=Path)
    args = parser.parse_args()
    stage(args.batch_workspace.resolve(), args.new_workspace.resolve())
