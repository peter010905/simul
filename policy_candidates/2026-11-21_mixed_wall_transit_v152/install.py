"""Copy frozen v1.5.1 to an independent opt-in wall-transit workspace."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def replace(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Unexpected frozen anchor: '+before[:100])
    return text.replace(before, after, 1)


def install(source, destination):
    if destination.exists():
        raise FileExistsError(destination)
    here = Path(__file__).resolve().parent
    package = destination/'src/osp_mission'
    package.parent.mkdir(parents=True)
    shutil.copytree(source/'src/osp_mission', package,
        ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache'))
    modules = package/'osp_mission'
    mission = modules/'mission.py'
    text = replace(mission.read_text(), 'from osp_mission.near_pass_runtime import NearPassRuntime',
        'from osp_mission.wall_transit_runtime import WallTransitRuntime as NearPassRuntime')
    text = replace(text, 'self.near_pass.inspectable_route=False  # The phase scheduler owns surface observations.',
        'self.near_pass.inspectable_route=inspectable_route(selected)')
    callback = "        if getattr(self,'near_pass',None) is not None and self.near_pass.completed(result.status,self.kind,getattr(self,'cancel_reason','')):return\n"
    text = replace(text, callback, '')
    text = replace(text, "        if self.cancel_reason == 'readable_marker' and result.status in (4,5):",
        callback+"        if self.cancel_reason == 'readable_marker' and result.status in (4,5):")
    mission.write_text(text)
    schedule = modules/'center_wall.py'
    text = replace(schedule.read_text(), '        if not boxes:\n            goal = self.weak.choose',
        '        wall_reserve = min(.15*duration, 12. + 5.*self.counts[\'wall\'])\n'
        '        if not boxes or duration-elapsed <= wall_reserve:\n            goal = self.weak.choose')
    text = replace(text, '        if crowded and not boxes:',
        "        if candidates['wall'] and duration-elapsed <= wall_reserve:\n"
        "            self.phase, self.reason = 'wall', 'remaining_time_for_mapped_walls'\n"
        "            self.idle_since = None\n"
        "            return min(candidates['wall'], key=lambda item: item[0])[1]\n"
        '        if crowded and not boxes:')
    text = replace(text, "        wall_reserve = min(.15*duration, 12. + 5.*self.counts['wall'])\n        frontier = None if fresh",
        '        frontier = None if fresh')
    schedule.write_text(text)
    for name in ('wall_transit_inspection.py', 'wall_transit_runtime.py'):
        shutil.copy2(here/name, modules/name)
    hashes = {}
    for path in (source/'src/osp_mission').rglob('*'):
        if not path.is_file() or any(p in path.parts for p in ('__pycache__', '.pytest_cache')):
            continue
        relative = path.relative_to(source/'src/osp_mission')
        if str(relative) not in ('osp_mission/mission.py', 'osp_mission/center_wall.py'):
            assert path.read_bytes() == (package/relative).read_bytes(), relative
            hashes[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
    (destination/'wall_transit_manifest.json').write_text(json.dumps(dict(version='1.5.2-wall-transit-r2',
        base='1.5.1-balanced', unchanged_sha256=hashes,
        overlay_sha256={name:hashlib.sha256((modules/name).read_bytes()).hexdigest()
            for name in ('mission.py', 'center_wall.py', 'wall_transit_inspection.py', 'wall_transit_runtime.py')},
        changed=['osp_mission/mission.py', 'osp_mission/center_wall.py'],
        added=['osp_mission/wall_transit_inspection.py', 'osp_mission/wall_transit_runtime.py'],
        collision_parameters_changed=False, recognition_parameters_changed=False,
        requires_allow_mid_route_wall_inspection=True), indent=2)+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    a = p.parse_args()
    install(a.source.resolve(), a.destination.resolve())
