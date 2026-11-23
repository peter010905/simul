"""Use the current safe pose for a nearby visible wall on a separate Focus Continuity stage."""
import argparse
import importlib.util
import json
import subprocess
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'local_print_route', HERE.parent / '2026-11-24_v152_local_print_route/install.py')
parent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parent)
digest = parent.digest


def install(focus, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'stage'
        parent.install(focus, stage)
        package = stage / 'src/osp_mission/osp_mission'
        subprocess.run(['patch', '-p1', '-F', '0', '-i', str(HERE / 'observe_here.patch')],
                       cwd=package, check=True)
        changed = ('center_wall.py', 'phased_hybrid.py', 'mission.py')
        for name in changed:
            compile((package / name).read_text(), name, 'exec')
        manifest = dict(version='1.5.2.24-observe-here',
                        parent_sha256=digest(stage / 'local_print_route_manifest.json'),
                        sources={name: digest(package / name) for name in changed})
        (stage / 'observe_here_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('focus', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.focus, args.destination)
