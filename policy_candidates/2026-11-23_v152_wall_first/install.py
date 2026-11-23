"""Stage the wall-first policy from the v1.5.2.1 Focus Continuity workspace."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
PARENT = {
    'center_wall.py': '013cc8432c4a14aaf47b25d2672d94c5117d0351fd8c207d990e56ccf4028fac',
    'phased_hybrid.py': 'b962de92d15726667f876293f78bad889408c2388f9c6f46aeb4bb067bd06e90',
    'mission.py': '6c43c48ed2599b0d468bdededc5e2f9f985ec08158150676c77e032fc078a821',
}


def stage(parent, destination):
    parent = parent.expanduser().resolve()
    destination = destination.expanduser().resolve()
    if destination.exists():
        raise FileExistsError(destination)
    for name, digest in PARENT.items():
        if hashlib.sha256((parent/'src/osp_mission/osp_mission'/name).read_bytes()).hexdigest() != digest:
            raise ValueError(f'Expected Focus Continuity source for {name}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temporary:
        work = Path(temporary)/'workspace'
        shutil.copytree(parent/'src', work/'src', symlinks=True)
        package = work/'src/osp_mission'
        subprocess.run(['patch', '-F', '0', '-p1', '-i', str(HERE/'wall_first.patch')],
                       cwd=package, check=True)
        shutil.copy2(HERE/'wall_first.py', package/'osp_mission/wall_first.py')
        work.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    stage(args.parent, args.destination)
