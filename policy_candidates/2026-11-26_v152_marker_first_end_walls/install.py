"""Fork small-return filter with marker-first scheduling and automatic opposing wall suppression."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'center_wall.py': (
        ('        self.chair_band_contains = lambda point: False\n',
         '        self.chair_band_contains = lambda point: False\n'
         '        self.end_wall_surface = lambda point, yaw: False\n'),
        ("        candidates['wall'] = [item for item in weak_walls if item[1]['wall_coverage_possible']]\n",
         "        ordinary_walls = [item for item in weak_walls if not self.end_wall_surface(\n"
         "            item[1]['surface_map'], item[1]['normal_odom']-transform[2])]\n"
         "        candidates['wall'] = [item for item in ordinary_walls if item[1]['wall_coverage_possible']]\n"),
        ("        self.counts['glass_wall_skipped'] = self.glass_wall.skipped_groups\n",
         "        self.counts['glass_wall_skipped'] = self.glass_wall.skipped_groups\n"
         "        self.counts['automatic_end_wall_skipped'] = len(weak_walls)-len(ordinary_walls)\n"),
    ),
    'hybrid.py': (
        ('    def should_interrupt(self,owner):\n',
         '    def has_fresh_print(self, owner):\n'
         '        stamp = owner.get_clock().now().nanoseconds/1e9\n'
         "        return any(row['count'] >= 3 and 0 <= stamp-row['stamp'] <= 1.\n"
         '                   for row in self.memory.tracks)\n\n'
         '    def should_interrupt(self,owner):\n'),
        ("        stamp=owner.get_clock().now().nanoseconds/1e9\n"
         "        fresh_print = any(r['count']>=3 and 0<=stamp-r['stamp']<=1. for r in self.memory.tracks)\n",
         "        fresh_print = self.has_fresh_print(owner)\n"),
        ('        if survey is not None and survey.can_continue(now-owner.started):\n',
         '        if not fresh_print and survey is not None and survey.can_continue(now-owner.started):\n'),
    ),
    'mission.py': (
        ("            if (hybrid is not None and self.kind=='nav' and not protected and not self.cancel_reason\n",
         "            if (hybrid is not None and self.kind=='nav' and not self.cancel_reason\n"
         "                    and (not protected or hybrid.has_fresh_print(self))\n"),
        ("                    and not getattr(self.near_pass,'protected_navigation',False)\n"
         "                    and hybrid.should_interrupt(self)):\n",
         "                    and not getattr(hybrid,'active_print_priority',False)\n"
         "                    and hybrid.should_interrupt(self)):\n"),
    ),
    'phased_hybrid.py': (
        ('from osp_mission.auto_chair_strip import ChairCueFilter\n',
         'from osp_mission.auto_chair_strip import ChairCueFilter\n'
         'from osp_mission.end_walls import EndWalls\n'),
        ('        self.schedule.chair_band_contains = chair_strip.band_contains\n'
         "        if selected is not None and chair_strip.band_contains((selected['x'], selected['y'])):\n",
         '        ends = EndWalls(grid)\n'
         '        self.schedule.end_wall_surface = ends.wall_surface\n'
         '        def reject_exploration(point):\n'
         '            return chair_strip.band_contains(point) or ends.contains(point)\n'
         '        self.schedule.chair_band_contains = reject_exploration\n'
         "        if selected is not None and reject_exploration((selected['x'], selected['y'])):\n"),
        ("        if goal is not None and ('routed_approach_m' in goal or 'box_recovery_walk_m' in goal):\n"
         "            return goal\n"
         "        protected = goal is not None and (goal.get('weak_wall') or 'bearing_advance_id' in goal or goal.get('joint_continuation')\n"
         "            or any(k in goal for k in ('distant_hint_id', 'survey_escape'))\n"
         "            or ('survey_wall' in goal and 'joint_views' not in goal))\n"
         "        if cue is not None and not protected:\n",
         "        if goal is not None and 'routed_approach_m' in goal:\n"
         "            return goal\n"
         "        if cue is not None:\n"),
        ('        super().dispatched(owner,goal)\n',
         '        super().dispatched(owner,goal)\n'
         "        self.active_print_priority = any(k in goal for k in (\n"
         "            'approach_id', 'focus_id', 'print_point', 'routed_approach_m'))\n"),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'small_return_filter_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.37-small-return-filter':
        raise ValueError('Expected Small Return Filter parent')
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
        helper = 'end_walls.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'marker_first_end_walls_manifest.json').write_text(json.dumps({
            'version': '1.5.2.38-marker-first-end-walls',
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
