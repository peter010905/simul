"""Evaluator-only trajectory/repeated-goal analysis; never imported by a policy."""
import argparse
import csv
import json
import math
from pathlib import Path
import re


def analyze(directory):
    points = list(csv.DictReader((directory/'ground_truth_trajectory.csv').open()))
    previous = None
    history = {}
    distance = repeated = 0.
    reentries = 0
    current_cell = None
    returning = False
    for row in points:
        x, y, stamp = float(row['world_x']), float(row['world_y']), float(row['sim_seconds'])
        if not all(math.isfinite(v) for v in (x,y,stamp)):
            continue
        if previous is not None:
            segment = math.hypot(x-previous[0],y-previous[1])
            if segment < .2:
                continue
            if segment > 1.5:
                previous = (x,y)
                continue
        else:
            segment = 0.
        cell = (math.floor(x/.6),math.floor(y/.6))
        if cell != current_cell:
            returning = cell in history and stamp-history[cell]>15
            reentries += int(returning)
            current_cell = cell
        history[cell] = stamp
        distance += segment
        if returning:
            repeated += segment
        previous = (x,y)
    log = (directory/'launch.log').read_text(errors='replace')
    goals = [tuple(map(float,pair)) for pair in re.findall(r'Navigation goal \(([-.\d]+), ([-.\d]+)\)',log)]
    repeated_goals = sum(any(math.dist(point,old)<.8 for old in goals[:max(0,i-2)]) for i,point in enumerate(goals))
    return dict(episode=str(directory), source='Independent Gazebo ground-truth trajectory; evaluator only',
        path_distance_m=distance, distance_in_returned_cells_m=repeated,
        returned_distance_fraction=repeated/distance if distance else 0,
        cell_reentries_after_15_sim_seconds=reentries, cell_size_m=.6, minimum_path_step_m=.2,
        navigation_goals=len(goals), repeated_map_goals_within_0_8m_excluding_last_two=repeated_goals,
        limits=['Returned cells include necessary corridor transit and marker approach; this is not waste distance',
                'Repeated goals use SLAM map coordinates and may be affected by frame correction',
                'These are simulation measurements, not observations from a physical robot'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode',type=Path)
    parser.add_argument('--output',type=Path)
    args = parser.parse_args()
    result = json.dumps(analyze(args.episode.resolve()),indent=2)+'\n'
    if args.output:
        args.output.write_text(result)
    else:
        print(result)
