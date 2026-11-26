"""Fork Box Safe Views with resumable exploration trips."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    '../launch/robot.launch.py': (
        ("        if value('forward_045') == 'true':\n",
         """        if not sim:
            layer = nav_config['local_costmap']['local_costmap']['ros__parameters'].get('voxel_layer', {})
            if layer.get('plugin') == 'nav2_costmap_2d::VoxelLayer' and layer.get('z_voxels') == 16:
                layer['z_resolution'] = max(layer.get('z_resolution', .05), .075)
        if value('forward_045') == 'true':
"""),
    ),
    'center_wall.py': (
        ('from osp_mission.box_batch import BoxBatch\n',
         'from osp_mission.box_batch import BoxBatch\nfrom osp_mission.region_transit import RegionTransit\n'),
        ('        self.batch = BoxBatch()\n',
         '        self.batch = BoxBatch()\n        self.region_transit = RegionTransit()\n'),
        ('        if not probe_only:\n            recovery = self.box_recovery.choose',
         '''        robot = odom_point(pose[:2], transform)
        mode = None if probe_only else self.region_transit.mode(elapsed, robot, self.survey.trail)
        if not probe_only and mode != 'resume':
            recovery = self.box_recovery.choose'''),
        ("        if alternative is not None and (reserved is None or self.survey.loop_center is not None or urgent_wall):",
         """        if alternative is not None and ('distant_hint_id' in alternative
                or (mode != 'resume' and reserved is None)
                or (not mode and (self.survey.loop_center is not None or urgent_wall))):"""),
        ('        if center is not None and elapsed-self.survey.last_escape >= 35.:',
         '        if center is not None and not mode and elapsed-self.survey.last_escape >= 35.:'),
        ('        if not crowded:\n            frontier = discovery_frontier',
         '''        if mode and (mode == 'resume' or not local_box):
            views = choices
            if (reserved is not None and not self.chair_band_contains((reserved['x'], reserved['y']))
                    and not self.glass_wall.includes((reserved['x'], reserved['y']), .8)):
                views = (*choices, reserved)
            transit = self.region_transit.choose(mode, views, grid, box_distances, transform,
                robot, self.survey.visited_positions(elapsed), duration-elapsed, veto)
            if transit is not None:
                self.phase = 'wall' if 'survey_wall' in transit else 'interior'
                self.reason = 'region_' + mode
                return transit
            if reserved is not None and (urgent_wall or self.survey.loop_center is not None):
                self.phase, self.reason = 'wall', 'unobserved_wall_reserved'
                return reserved
        if not crowded:
            frontier = discovery_frontier'''),
        ("        if not local_box:\n            closer = self.bearing_advance.choose",
         """        if mode == 'resume' or not local_box:
            closer = self.bearing_advance.choose"""),
        ("        if 'current_wall_pose_odom' in goal:\n",
         "        self.region_transit.dispatched(goal, elapsed, self.joint.robot_xy)\n"
         "        if 'current_wall_pose_odom' in goal:\n"),
    ),
    'regional_surface.py': (
        ('        continuing = bool(local and (same_box or local[0][0] >= .78 * score))',
         '''        competing_score = max(score, best_frontier[0]) if best_frontier else score
        continuing = bool(local and (same_box or local[0][0] >= .78 * competing_score))'''),
    ),
    'phased_hybrid.py': (
        ('        if print_only and (cue is not None or not box_probe):',
         '        if print_only and (cue is not None or not box_probe or self.schedule.region_transit.active):'),
        ("        if cue is not None:\n            self.schedule.reason",
         """        if cue is not None and cue.get('white_leg') and goal is not None and any(
                key in goal for key in ('bearing_advance_id','distant_hint_id','weak_wall')):
            return goal
        if cue is not None:
            self.schedule.reason"""),
        ('        if goal is not None:\n            return goal\n        return selected',
         '''        if goal is not None:
            if goal.get('region_action') == 'resume':
                goal['region_destination_odom'] = list(self.schedule.region_transit.anchor)
            elif (not print_only and (goal.get('r32_kind') == 'interior_frontier' or goal.get('region_action') == 'leave')
                    and np.hypot(goal['x']-pose[0], goal['y']-pose[1]) >= 1.5):
                goal = dict(goal, region_destination_odom=odom_point((goal['x'],goal['y']), transform).tolist())
            return goal
        return selected'''),
        ('                   cluster_exit=bool', "                   region_action=goal.get('region_action'), region_destination_odom=goal.get('region_destination_odom'),\n                   cluster_exit=bool"),
    ),
    'mission.py': (
        ("        self.cancel_reason = reason\n        self.get_logger().warning",
         """        self.cancel_reason = reason
        schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
        if schedule is not None:
            schedule.region_transit.interrupt(reason)
        self.get_logger().warning"""),
    ),
    'hybrid.py': (
        ('        self.used = []\n',
         '        self.used = []\n        self.white_used = []\n'),
        ("        self.used=[r for r in self.used if now-r[1]<180]\n",
         "        self.used=[r for r in self.used if now-r[1]<180]\n"
         "        self.white_used=[r for r in self.white_used if now-r[1]<180]\n"),
        ("and not any(np.linalg.norm(r['point']-p)<.75 for p,_ in self.used)",
         """and not any(np.linalg.norm(r['point']-p)<.75 for p,_ in self.used)
                and not (r.get('white_leg') and any(np.linalg.norm(r['point']-p)<.75 for p,_ in self.white_used))"""),
        ("min(reachable, key=lambda row: row[0])",
         "min(reachable, key=lambda row: (row[1].get('white_leg', False), row[0]))"),
        ("self.memory.used.append((np.asarray(goal['print_odom']),owner.get_clock().now().nanoseconds/1e9))",
         """used = self.memory.white_used if goal.get('white_leg') else self.memory.used
            used.append((np.asarray(goal['print_odom']), owner.get_clock().now().nanoseconds/1e9))"""),
    ),
}

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'box_safe_views_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.40-box-safe-views':
        raise ValueError('Expected Box Safe Views parent')
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
        helper = 'region_transit.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'region_transit_manifest.json').write_text(json.dumps({
            'version': '1.5.2.41-region-transit',
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
