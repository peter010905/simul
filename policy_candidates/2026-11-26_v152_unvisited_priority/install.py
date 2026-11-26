"""Fork marker-first exploration with higher scores for useful unvisited regions."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tempfile


PACKAGE = Path('src/osp_mission/osp_mission')
PATCHES = {
    'center_wall.py': (
        ("            combined = self.joint.choose(grid, reachable, distances, transform, self.survey,\n"
         "                                         candidates['wall'], fresh or candidates['interior'],\n"
         "                                         reserved, elapsed, duration, frontier=frontier, heading=pose[2])\n"
         "            if (not (combined and combined.get('joint_continuation'))\n"
         "                    and elapsed-self.last_new_region >= 90. and duration-elapsed >= 55.):\n"
         "                region = choose_unvisited(choices, grid, costmap, nav, reachable, distances, transform,\n"
         "                                          self.survey.trail, elapsed, duration-elapsed, veto,\n"
         "                                          visited_xy=self.survey.visited_positions(elapsed))\n"
         "                if region is not None:\n"
         "                    self.phase, self.reason = 'interior', 'open_new_region_for_camera_coverage'\n"
         "                    return region\n",
         "            region = None\n"
         "            if elapsed-self.last_new_region >= 22. and duration-elapsed >= 35.:\n"
         "                region = choose_unvisited(choices, grid, costmap, nav, reachable, distances, transform,\n"
         "                                          self.survey.trail, elapsed, duration-elapsed, veto,\n"
         "                                          visited_xy=self.survey.visited_positions(elapsed))\n"
         "            combined = self.joint.choose(grid, reachable, distances, transform, self.survey,\n"
         "                                         candidates['wall'], fresh or candidates['interior'],\n"
         "                                         reserved, elapsed, duration, frontier=frontier,\n"
         "                                         heading=pose[2], new_region=region)\n"
         "            if combined is None and region is not None:\n"
         "                self.phase, self.reason = 'interior', 'open_new_region_for_camera_coverage'\n"
         "                return region\n"),
        ("                self.reason = ('discover_beyond_current_region' if combined['r32_kind'] == 'interior_frontier' else\n",
         "                self.reason = ('open_new_region_for_camera_coverage' if combined.get('survey_region') else\n"
         "                               'discover_beyond_current_region' if combined['r32_kind'] == 'interior_frontier' else\n"),
    ),
    'regional_surface.py': (
        ('from osp_mission.joint_surface import View, inspection_key, projected_coverage\n',
         'from osp_mission.joint_surface import View, inspection_key, projected_coverage\n'
         'from osp_mission.unvisited_priority import VisitNovelty\n'),
        ('walls, interiors, reserved, elapsed, duration, *, frontier=None, heading=None):',
         'walls, interiors, reserved, elapsed, duration, *, frontier=None, heading=None, new_region=None):'),
        ('        remaining = duration - elapsed\n',
         '        remaining = duration - elapsed\n'
         '        novelty = VisitNovelty(survey.visited_positions(elapsed))\n'
         '        feasible_frontiers = []\n'
         '        for candidate in (frontier, new_region):\n'
         '            if candidate is None:\n'
         '                continue\n'
         "            cell = grid.index(candidate['x'], candidate['y'])\n"
         '            if grid.inside(*cell) and safe[cell] and 0 <= distances[cell] <= .20*(remaining-8):\n'
         '                walk = float(distances[cell])\n'
         '                feasible_frontiers.append((novelty.frontier_score(candidate, transform, walk),\n'
         '                                           novelty.annotate(candidate, transform, walk)))\n'
         '        best_frontier = max(feasible_frontiers, key=lambda item: item[0], default=None)\n'),
        ('        if not views:\n            return None\n',
         '        if not views:\n            return best_frontier[1] if best_frontier is not None else None\n'),
        ('            score = gain / (1. + .45 * view.walk + .10 * turn + repeat_cost)\n',
         "            distance = novelty.distance(view.goal, transform)\n"
         "            follow_box = (active and same_area and view.goal.get('local_batch')\n"
         "                          and not view.goal.get('completed_box') and not view.goal.get('previous_attempts'))\n"
         "            if not follow_box:\n"
         "                gain *= novelty.new_gain(distance)\n"
         "                repeat_cost += novelty.return_cost(distance, view.walk)\n"
         "            score = gain / (1. + .45 * view.walk + .10 * turn + repeat_cost)\n"),
        ("        if not ranked:\n"
         "            if frontier is not None:\n"
         "                cell = grid.index(frontier['x'], frontier['y'])\n"
         "                if grid.inside(*cell) and safe[cell] and 0 <= distances[cell] <= .20*(remaining-8):\n"
         "                    return frontier\n"
         "            return None\n",
         "        if not ranked:\n"
         "            return best_frontier[1] if best_frontier is not None else None\n"),
        ("        elif frontier is not None:\n"
         "            # Frontier gain uses a different unit than camera coverage. Only\n"
         "            # promote a reachable, informative frontier once the current area\n"
         "            # has no worthwhile continuation and the surface is a repeat/long leg.\n"
         "            target = grid.index(frontier['x'], frontier['y'])\n"
         "            if grid.inside(*target) and safe[target]:\n"
         "                walk = float(distances[target])\n"
         "                gain = float(frontier['features'][1])\n"
         "                frontier_score = min(2., 1.15 + gain) / (1. + .45 * walk)\n"
         "                if (0 <= walk <= .20 * (remaining - 8) and frontier_score > score\n"
         "                        and (selected.walk > 2.3 or ranked[0][3])):\n"
         "                    return frontier\n"
         "        goal = dict(selected.goal, joint_views=len(seen), joint_walk_m=round(selected.walk, 3),\n",
         "        elif best_frontier is not None and best_frontier[0] > score:\n"
         "            return best_frontier[1]\n"
         "        selected_goal = novelty.annotate(selected.goal, transform, selected.walk)\n"
         "        goal = dict(selected_goal, joint_views=len(seen), joint_walk_m=round(selected.walk, 3),\n"),
    ),
    'phased_hybrid.py': (
        ("                   joint_continuation=goal.get('joint_continuation'),\n",
         "                   joint_continuation=goal.get('joint_continuation'),\n"
         "                   visit_novelty_m=goal.get('visit_novelty_m'),\n"
         "                   visit_novelty_gain=goal.get('visit_novelty_gain'),\n"
         "                   visit_return_cost=goal.get('visit_return_cost'),\n"),
    ),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(parent, destination):
    parent, destination = parent.resolve(), destination.resolve()
    manifest = parent / 'marker_first_end_walls_manifest.json'
    source = json.loads(manifest.read_text())
    if source['version'] != '1.5.2.38-marker-first-end-walls':
        raise ValueError('Expected Marker First End Walls parent')
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
        helper = 'unvisited_priority.py'
        shutil.copyfile(Path(__file__).with_name(helper), stage / PACKAGE / helper)
        (stage / 'unvisited_priority_manifest.json').write_text(json.dumps({
            'version': '1.5.2.39-unvisited-priority',
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
