"""Install the same-box tour candidate over Mixed v1.5.2.11."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
PARENT = {'center_wall.py': '83dcfa3b030f5ec53baff9b06e5541524cddb79a1fce99a83aeefc1f93a31f75',
          'phased_hybrid.py': 'b962de92d15726667f876293f78bad889408c2388f9c6f46aeb4bb067bd06e90'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def edit(path, changes):
    source = path.read_text()
    for current, replacement in changes:
        if source.count(current) != 1:
            raise ValueError(f'Unexpected integration point in {path.name}: {current}')
        source = source.replace(current, replacement, 1)
    compile(source, str(path), 'exec')
    path.write_text(source)


def install(release, destination):
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    parent = HERE.parent/'2026-11-23_v152_weak_wall_priority/install.py'
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp:
        stage = Path(temp)/'stage'
        subprocess.run([sys.executable,str(parent),str(release.resolve()),str(stage)],check=True)
        package = stage/'src/osp_mission/osp_mission'
        for name, expected in PARENT.items():
            if digest(package/name) != expected:
                raise ValueError(f'Unexpected v1.5.2.11 source: {name}')
        shutil.copy2(HERE/'box_tour.py', package)
        edit(package/'center_wall.py', [
            ('from osp_mission.box_batch import BoxBatch',
             'from osp_mission.box_batch import BoxBatch\nfrom osp_mission.box_tour import BoxTour'),
            ('        self.batch = BoxBatch()', '        self.batch = BoxBatch()\n        self.tour = BoxTour()'),
            ("        local_box = next((goal for _, goal in fresh if goal.get('local_batch')), None)",
             "        tour_goal = self.tour.choose(fresh, grid, reachable, odom_point(pose[:2],transform), elapsed, duration)\n"
             "        if tour_goal is not None:\n"
             "            self.phase, self.reason = 'interior', 'finish_reachable_box_faces_in_order'\n"
             "            return tour_goal\n"
             "        local_box = next((goal for _, goal in fresh if goal.get('local_batch')), None)"),
            ('        self.batch.dispatched(goal)',
             '        self.batch.dispatched(goal)\n        self.tour.dispatched(goal, elapsed)'),
        ])
        edit(package/'phased_hybrid.py', [
            ("                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),",
             "                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),\n"
             "                   box_tour_order=goal.get('box_tour_order'), box_tour_route_m=goal.get('box_tour_route_m'),"),
        ])
        manifest = dict(version='1.5.2.13-box-tour',
                        parent_sha256=digest(stage/'weak_wall_priority_manifest.json'),
                        sources={name:digest(package/name) for name in (*PARENT,'box_tour.py')})
        (stage/'box_tour_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('release',type=Path)
    parser.add_argument('destination',type=Path)
    args = parser.parse_args()
    install(args.release,args.destination)
