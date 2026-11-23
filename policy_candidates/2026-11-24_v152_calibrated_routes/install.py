"""Build calibrated route selection on a separate Focus Continuity stage."""
import argparse
import importlib.util
import json
import shutil
import subprocess
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'finish_transit', HERE.parent / '2026-11-24_v152_finish_transit/install.py')
parent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parent)
digest = parent.open_regions.stable.regional.joint.digest


def install(focus, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        stage = Path(tmp) / 'stage'
        parent.install(focus, stage)
        package = stage / 'src/osp_mission/osp_mission'
        subprocess.run(['patch', '-p1', '-F', '0', '-i', str(HERE / 'routes.patch')],
                       cwd=package, check=True)
        shutil.copy2(HERE / 'grid_travel.py', package)
        changed = ('grid_travel.py', '_r32_navigation_core.py', 'wall_bearings.py',
                   'center_wall.py', 'phased_hybrid.py')
        for name in changed:
            compile((package / name).read_text(), name, 'exec')
        manifest = dict(version='1.5.2.19-calibrated-routes',
                        parent_sha256=digest(stage / 'finish_transit_manifest.json'),
                        sources={name: digest(package / name) for name in changed})
        (stage / 'calibrated_routes_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('focus', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.focus, args.destination)
