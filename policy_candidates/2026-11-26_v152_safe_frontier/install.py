"""Fork Astra v1.5.2.41 with safer frontier follow-through."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'center_wall.py': (
        ('from osp_mission.region_transit import RegionTransit\n',
         'from osp_mission.region_transit import RegionTransit, route_connected\n'),
        ('        self.region_transit = RegionTransit()\n',
         '        self.region_transit = RegionTransit()\n        self.exploration_routes = None\n'),
        ('        if transform is None or viewed is None or viewed.shape != (4,)+grid.cells.shape:',
         '        self.exploration_routes = None\n'
         '        if transform is None or viewed is None or viewed.shape != (4,)+grid.cells.shape:'),
        ("        candidates = {'interior': [], 'wall': []}\n",
         """        self.exploration_routes = box_distances
        choices = tuple(goal for goal in choices if route_connected(grid, box_distances, goal))
        candidates = {'interior': [], 'wall': []}
"""),
        ('        if not probe_only:\n            recovery = self.box_recovery.choose',
         """        robot = odom_point(pose[:2], transform)
        mode = None if probe_only else self.region_transit.mode(elapsed, robot, self.survey.trail)
        if not probe_only and mode != 'resume':
            recovery = self.box_recovery.choose"""),
        ("""        robot = odom_point(pose[:2], transform)
        mode = self.region_transit.mode(elapsed, robot, self.survey.trail)
        if alternative is not None and (reserved is None or
""",
         """        if alternative is not None and ('distant_hint_id' in alternative
                or (mode != 'resume' and reserved is None) or
"""),
        ('        if not local_box:\n            closer =',
         "        if mode == 'resume' or not local_box:\n            closer ="),
        ('        if mode and not local_box:',
         "        if mode and (mode == 'resume' or not local_box):"),
    ),
    'phased_hybrid.py': (
        ('from osp_mission.r32_drive import grid_from_message\n',
         'from osp_mission.r32_drive import grid_from_message\n'
         'from osp_mission.region_transit import route_connected\n'),
        ('        if print_only and (cue is not None or not box_probe):',
         '        if print_only and (cue is not None or not box_probe or self.schedule.region_transit.active):'),
        ("""        if goal is not None:
            if goal.get('region_action') == 'resume':
""",
         """        if print_only:
            return goal
        routes = self.schedule.exploration_routes
        if goal is not None and not route_connected(grid, routes, goal):
            goal = None
        if selected is not None and not route_connected(grid, routes, selected):
            self.band_rejected_selected = True
            selected = None
        if goal is None and selected is not None and selected['features'][1] >= .15 and not selected.get('stationary'):
            self.schedule.reason = 'safe_frontier_fallback'
            goal = dict(selected, r32_kind='interior_frontier')
        if goal is not None:
            if goal.get('region_action') == 'resume':
"""),
        ("goal.get('r32_kind') == 'interior_frontier' or goal.get('region_action') == 'leave'",
         "goal.get('r32_kind') in ('interior_frontier', 'unvisited_region') or goal.get('region_action') == 'leave'"),
    ),
    'mission.py': (
        ("            if self.kind=='nav':\n                self.excluded.append(self.destination)\n                flow=getattr(self,'navigation_flow',None)\n                if flow is not None:flow.record_navigation_result(6,'action_rejected',self.now())",
         """            if self.kind=='nav':
                self.excluded.append(self.destination)
                flow=getattr(self,'navigation_flow',None)
                if flow is not None:flow.record_navigation_result(6,'action_rejected',self.now())
                schedule = getattr(getattr(self, 'hybrid', None), 'schedule', None)
                if schedule is not None:
                    schedule.region_transit.navigation_finished()"""),
        ('                schedule.box_recovery.navigation_result(result.status, self.cancel_reason, self.now()-self.started)',
         '                schedule.box_recovery.navigation_result(result.status, self.cancel_reason, self.now()-self.started)\n'
         '                schedule.region_transit.navigation_finished()'),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'region_transit_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.41-region-transit':
        raise ValueError('Expected Astra v1.5.2.41 region transit parent')
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
        (stage / 'safe_frontier_manifest.json').write_text(json.dumps({
            'version': '1.5.2.42-safe-frontier',
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
