"""Fork unvisited exploration with box viewpoints that account for occluders and the rolling costmap."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'adaptive_faces.py': (
        ('pose, goal_fields, veto, visibility_offset=.06):',
         'pose, goal_fields, veto, visibility_offset=.06, *, allow_side=False):'),
        ('    stand, shift = np.meshgrid(np.arange(.40, 1.501, step), np.arange(-.50, .501, step*2))\n',
         '    side_limit = .80 if allow_side else .50\n'
         '    stand, shift = np.meshgrid(np.arange(.40, 1.501, step),\n'
         '                               np.arange(-side_limit, side_limit+.001, step*2))\n'),
        ('    valid = (length <= 1.5) & (abs(shift) <= .75*stand) & (half_angle <= math.radians(28))\n',
         '    valid = (length <= 1.5) & (abs(shift) <= (.95 if allow_side else .75)*stand)\n'
         '    if not allow_side:\n'
         '        valid &= half_angle <= math.radians(28)\n'
         '    half_angle = half_angle[valid]\n'),
        ('        if not veto(goal) and grid.line_clear(point, face + visibility_offset*normal):\n'
         '            return float(distances[rows[i],cols[i]]), goal\n',
         '        if veto(goal):\n'
         '            continue\n'
         '        if half_angle[i] <= math.radians(28) and grid.line_clear(point, face + visibility_offset*normal):\n'
         '            return float(distances[rows[i],cols[i]]), goal\n'
         '        if not allow_side or half_width < .10:\n'
         '            continue\n'
         "        for fraction in sorted((-.60,.60), key=lambda f: abs(shift[i]-f*half_width)):\n"
         '            aim = face + fraction*half_width*tangent\n'
         '            camera_distance = np.linalg.norm(point-aim)\n'
         '            if camera_distance > 1.5 or float((point-aim)@normal) < .60*camera_distance:\n'
         '                continue\n'
         '            r, c = grid.index(*(aim-.04*normal))\n'
         '            support = grid.cells[max(0,r-2):r+3,max(0,c-2):c+3]\n'
         '            if not grid.inside(r,c) or not (support >= 65).any():\n'
         '                continue\n'
         '            if grid.line_clear(point, aim + visibility_offset*normal):\n'
         '                heading = math.atan2(aim[1]-point[1], aim[0]-point[0])\n'
         "                side_goal = dict(goal, yaw=heading-goal['scan_angle']/2, face_aim_map=aim.tolist())\n"
         '                if not veto(side_goal):\n'
         '                    return float(distances[rows[i],cols[i]]), side_goal\n'),
    ),
    'center_wall.py': (
        ('from osp_mission.box_batch import BoxBatch\n',
         'from osp_mission.box_batch import BoxBatch\nfrom osp_mission.box_access import box_routes\n'),
        ('        self.batch = BoxBatch()\n',
         '        self.batch = BoxBatch()\n        self.local_box_cost = None\n'),
        ('        clearance = distance_cells * grid.resolution\n',
         '        clearance = distance_cells * grid.resolution\n'
         '        box_safe, box_distances = box_routes(grid, costmap, nav, reachable, pose, transform, self.local_box_cost)\n'),
        ('view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       half_width, pose, fields, veto, visibility_offset=.12)',
         'view = inspection_pose(grid, costmap, nav, box_distances, clearance, face, normal,\n'
         '                                       half_width, pose, fields, veto, visibility_offset=.12, allow_side=True)'),
        ('view = partial_face_pose(grid, costmap, nav, distances, clearance, face, normal,\n',
         'view = partial_face_pose(grid, costmap, nav, box_distances, clearance, face, normal,\n'),
        ('view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,\n'
         '                                       width, pose, fields, veto)',
         'view = inspection_pose(grid, costmap, nav, box_distances, clearance, face, normal,\n'
         '                                       width, pose, fields, veto, allow_side=True)'),
        ('self.box_recovery.choose(grid, costmap, reachable, nav, distances, clearance,\n',
         'self.box_recovery.choose(grid, costmap, box_safe, nav, box_distances, clearance,\n'),
    ),
    'box_face_recovery.py': (
        ('visibility_offset=offset)\n','visibility_offset=offset, allow_side=True)\n'),
    ),
    'two_views.py': (
        ('                if distance <= 1.5 and similar(direction,pose[2],min(27,math.degrees(fov)/2-3)):\n',
         '                if (distance <= 1.5 and similar(direction,pose[2],min(27,math.degrees(fov)/2-3))\n'
         '                        and grid.line_clear(observer, face+.10*normal)):\n'),
    ),
    'mission.py': (
        ("            self.create_subscription(OccupancyGrid,'global_costmap/costmap',self.hybrid.drive.update_costmap,qos_profile_sensor_data)\n",
         "            self.create_subscription(OccupancyGrid,'global_costmap/costmap',self.hybrid.drive.update_costmap,qos_profile_sensor_data)\n"
         "            self.create_subscription(OccupancyGrid,'local_costmap/costmap',self.hybrid.update_local_cost,qos_profile_sensor_data)\n"),
    ),
    'phased_hybrid.py': (
        ('        self.schedule = CenterWall()\n',
         '        self.schedule = CenterWall()\n        self.local_cost = None\n'),
        ('    def select(self, owner, selected, data, viewed, pose, transform, *,\n',
         '    def update_local_cost(self, msg):\n'
         '        self.local_cost = msg\n\n'
         '    def select(self, owner, selected, data, viewed, pose, transform, *,\n'),
        ('        self.schedule.chair_band_contains = reject_exploration\n',
         '        self.schedule.chair_band_contains = reject_exploration\n'
         '        self.schedule.local_box_cost = None\n'
         '        local = self.local_cost\n'
         "        if local is not None and local.header.frame_id in ('odom', owner.p('map_frame')):\n"
         '            local_age = clock-Time.from_msg(local.header.stamp).nanoseconds/1e9\n'
         '            if -.1 <= local_age <= 1.5:\n'
         '                self.schedule.local_box_cost = (local.header.frame_id, grid_from_message(local))\n'),
        ("                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),\n",
         "                   surface=goal.get('surface_map'), fitted_face=goal.get('fitted_face'),\n"
         "                   face_aim_map=goal.get('face_aim_map'),\n"
         "                   local_box_cost_used=self.schedule.local_box_cost is not None,\n"),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'unvisited_priority_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.39-unvisited-priority':
        raise ValueError('Expected Unvisited Priority parent')
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
        helper = 'box_access.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'box_safe_views_manifest.json').write_text(json.dumps({
            'version': '1.5.2.40-box-safe-views',
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
