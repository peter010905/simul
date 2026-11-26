"""Fork auto chair strip with bounded recovery for box observation goals."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'center_wall.py': (
        ('from osp_mission.box_batch import BoxBatch\n',
         'from osp_mission.box_batch import BoxBatch\n'
         'from osp_mission.box_face_recovery import BoxFaceRecovery\n'),
        ('        self.batch = BoxBatch()\n',
         '        self.batch = BoxBatch()\n        self.box_recovery = BoxFaceRecovery()\n'),
        ('                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       half_width, pose, fields, veto, visibility_offset=.12)\n',
         '                fields[\'recovery_geometry\'] = (anchor.tolist(), normal_yaw, float(half_width), .12)\n'
         '                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       half_width, pose, fields, veto, visibility_offset=.12)\n'),
        ('                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       .5*len(cells)*grid.resolution, pose, fields, veto)\n',
         '                width = .5*len(cells)*grid.resolution\n'
         "                fields['recovery_geometry'] = (odom_point(face, transform).tolist(), normal_yaw, width, .06)\n"
         '                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       width, pose, fields, veto)\n'),
        ("        self.counts['interior'] = len(ordered)\n",
         "        self.counts['interior'] = len(ordered)\n"
         '        for _, goal in ordered:\n'
         "            if goal.get('batch_box') is not None:\n"
         "                goal['recovery_view_odom'] = odom_point((goal['x'], goal['y']), transform).tolist()\n"
         '        if not probe_only:\n'
         '            recovery = self.box_recovery.choose(grid, costmap, reachable, nav, distances, clearance,\n'
         '                pose, transform, veto, ordered, elapsed, duration, self.glass_wall.in_zone)\n'
         '            if recovery is not None:\n'
         "                self.phase = 'interior'\n"
         "                self.reason = ('box_retry_same_face' if recovery['box_recovery_same_face']\n"
         "                               else 'box_continue_next_face')\n"
         '                return recovery\n'),
        ('        self.batch.dispatched(goal)\n',
         '        self.batch.dispatched(goal)\n'
         '        self.box_recovery.record_pick(goal, elapsed)\n'
         '        self.box_recovery.dispatched(goal)\n'),
    ),
    'phased_hybrid.py': (
        ("        if goal is not None and 'routed_approach_m' in goal:\n",
         "        if goal is not None and ('routed_approach_m' in goal or 'box_recovery_walk_m' in goal):\n"),
        ("                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),\n",
         "                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),\n"
         "                   box_recovery_walk_m=goal.get('box_recovery_walk_m'),\n"
         "                   box_recovery_same_face=goal.get('box_recovery_same_face'),\n"),
    ),
    'mission.py': (
        ("self.near_pass.protected_navigation=any(k in selected for k in ('approach_id','focus_id','print_point'))",
         "self.near_pass.protected_navigation=any(k in selected for k in "
         "('approach_id','focus_id','print_point','box_recovery_walk_m'))"),
        ("            if flow is not None:flow.record_navigation_result(result.status,getattr(self,'cancel_reason',''),self.now())\n",
         "            if flow is not None:flow.record_navigation_result(result.status,getattr(self,'cancel_reason',''),self.now())\n"
         "            schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)\n"
         "            if schedule is not None:\n"
         "                schedule.box_recovery.navigation_result(result.status, self.cancel_reason, self.now()-self.started)\n"),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'auto_chair_strip_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.35-auto-chair-strip':
        raise ValueError('Expected Auto Chair Strip parent')
    for name, expected in source['sources'].items():
        if digest(parent / PACKAGE / name) != expected:
            raise ValueError(f'Parent file changed: {name}')
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        stage = Path(directory) / 'stage'
        stage.mkdir()
        for name in ('src', 'frozen-config'):
            shutil.copytree(parent / name, stage / name, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        for path in parent.glob('*_manifest.json'):
            shutil.copyfile(path, stage / path.name)
        for name, changes in PATCHES.items():
            path = stage / PACKAGE / name
            code = path.read_text()
            for old, new in changes:
                if code.count(old) != 1:
                    raise ValueError(f'Unexpected {name} source at {old.strip()}')
                code = code.replace(old, new)
            compile(code, str(path), 'exec')
            path.write_text(code)
        helper = 'box_face_recovery.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'box_face_recovery_manifest.json').write_text(json.dumps({
            'version': '1.5.2.36-box-face-recovery',
            'parent_sha256': digest(manifest),
            'sources': {name: digest(stage / PACKAGE / name) for name in [*PATCHES, helper]},
        }, indent=2) + '\n')
        stage.rename(destination)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('parent', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    install(args.parent, args.destination)
