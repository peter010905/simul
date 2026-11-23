"""Compare recent and mission-long destination novelty on recorded Gazebo SLAM decisions."""
import argparse
import json
from pathlib import Path

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.open_regions import choose_unvisited
from osp_mission.visual_survey import VisualSurvey


def compare(run):
    alignment = json.loads((run / 'audit.json').read_text())['world_from_map_initial_alignment']
    rotation = np.asarray(alignment['rotation'])
    origin = np.asarray(alignment['translation'])
    track = np.loadtxt(run / 'headings.csv', delimiter=',', skiprows=1)
    decisions = [json.loads(s) for s in (run / 'native/center_wall.jsonl').read_text().splitlines()]
    times = [d['elapsed'] for d in decisions if d['reason'] == 'open_new_region_for_camera_coverage']
    rows = []
    for snapshot in sorted((run / 'native/policy_decisions').glob('*.npz')):
        with np.load(snapshot) as data:
            t = float(data['elapsed'])
            if not any(abs(t - target) < .15 for target in times):
                continue
            base = data['origin'].tolist()
            grid = Grid(data['data'], float(data['resolution']), tuple(base if len(base) == 3 else [*base, 0.]))
            safe, distance = grid.reachable(data['robot'], .32)
            columns = ['x', 'y', 'yaw', 'scan_angle', 'surface_gain', 'frontier_gain', 'distance', 'turn', 'revisit']
            if data['candidate_columns'].tolist() != columns:
                raise ValueError(f'Unexpected candidate fields in {snapshot}')
            choices = [dict(x=x, y=y, yaw=yaw, scan_angle=sweep, features=row)
                       for x, y, yaw, sweep, *row in data['candidates']]
        survey = VisualSurvey()
        survey.travel_memory = ExplorationMemory()
        for stamp, x, y, _ in track[track[:, 0] < t]:
            xy = (np.array([x, y]) - origin) @ rotation
            survey.position(*xy, stamp)
            survey.travel_memory.observe(*xy, stamp)
        recent = np.array([v[1:] for v in survey.trail if 0 <= t-v[0] <= 200.])

        def select(visited):
            goal = choose_unvisited(choices, grid, grid, safe, safe, distance, (0., 0., 0.),
                                    survey.trail, t, 600.-t, lambda _: False, visited_xy=visited)
            if goal is None:
                return None
            point = np.array([goal['x'], goal['y']]) @ rotation.T + origin
            return dict(world_xy=point.tolist(), novelty_m=goal['new_region_distance_m'],
                        estimated_walk_m=goal['new_region_path_m'])

        rows.append(dict(elapsed=t, recent=select(recent), mission=select(survey.visited_positions(t))))
    return dict(run=run.name, scope='Recorded SLAM mask, approximate odometry alignment; '
                'historical Nav2 costmap and exclusions unavailable.', snapshots=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, nargs='+')
    args = parser.parse_args()
    print(json.dumps([compare(run) for run in args.run], indent=2))
