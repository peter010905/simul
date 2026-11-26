"""Kinematic wall-lap comparison on synthetic rooms (no ROS nodes, no Nav2).

The robot is teleported along straight segments to each dispatched lap goal.
It measures, for the lap goal sequence only: backtracks (a goal behind the
previous actual travel direction), revisits (<0.8 m of an earlier goal), and
the in-place turn between each arrival heading and the next travel direction;
it does not prove Nav2 success, camera coverage, or physical safety.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from osp_mission._r32_navigation_core import Grid


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.WallLap


def room(kind, resolution=.05):
    w, h = (9., 7.)
    cells = np.zeros((round(h/resolution), round(w/resolution)), np.int8)
    t = 3
    cells[:t] = cells[-t:] = 100
    cells[:, :t] = cells[:, -t:] = 100
    rng = np.random.default_rng({'plain': 1, 'chairs': 2, 'dense': 3, 'notched': 4}[kind])
    if kind in ('chairs', 'dense'):
        # Rows of chair legs / seats protruding from the long walls.
        step = .9 if kind == 'chairs' else .6
        for x in np.arange(.8, w-.8, step):
            depth = rng.uniform(.25, .65)
            c0 = round(x/resolution)
            cells[t:t+round(depth/resolution), c0:c0+round(.35/resolution)] = 100
            cells[-t-round(depth/resolution):-t, c0:c0+round(.35/resolution)] = 100
    if kind == 'notched':
        for y in np.arange(1., h-1., 1.4):
            r0 = round(y/resolution)
            cells[r0:r0+round(.5/resolution), t:t+round(.9/resolution)] = 100
    cells[round(3./resolution):round(4./resolution), round(4./resolution):round(5./resolution)] = 100
    return Grid(cells, resolution, (0., 0., 0.))


def simulate(WallLap, grid, start, duration=600.):
    lap = WallLap()
    pose = (*start, 0.)
    t = 9.
    goals = []
    while lap.phase == 'perimeter' and t < duration and len(goals) < 60:
        safe, dist = grid.reachable(pose[:2], .32)
        goal = lap.choose(grid, dist, pose, (0., 0., 0.), t, duration, lambda _: False)
        if goal is None:
            t += 2.
            continue
        lap.dispatched(goal)
        target = np.array([goal['x'], goal['y']])
        here = np.array(pose[:2])
        n = max(1, int(np.linalg.norm(target-here)/.25))
        for k in range(1, n+1):
            p = here+(target-here)*k/n
            lap.position(float(p[0]), float(p[1]), t)
        t += float(goal['distance'])/.2+2.
        move = target-here
        goals.append((goal['x'], goal['y'], goal['yaw'], math.atan2(move[1], move[0])))
        pose = (goal['x'], goal['y'], goal['yaw'])
    backtracks = revisits = reversed_headings = 0
    turn_at_stops = 0.
    for i in range(1, len(goals)):
        # Arrival heading vs. the direction the robot moves next.
        turn = abs(math.remainder(goals[i][3]-goals[i-1][2], 2*math.pi))
        turn_at_stops += turn
        reversed_headings += turn > 3*math.pi/4
        if i >= 2:
            d = np.array(goals[i][:2])-np.array(goals[i-1][:2])
            if float(d @ [math.cos(goals[i-1][3]), math.sin(goals[i-1][3])]) < -.30:
                backtracks += 1
        if any(math.dist(goals[i][:2], g[:2]) < .8 for g in goals[:i-1]):
            revisits += 1
    return dict(goals=len(goals), backtracks=backtracks, revisits=revisits,
                reversed_headings=reversed_headings, turn_at_stops_rad=round(turn_at_stops, 2),
                path_m=round(lap.path, 2), turn_rad=round(lap.turn, 2), end_phase=lap.phase,
                end_reason=lap.reason, sim_elapsed_s=round(t, 1))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('parent', type=Path)
    p.add_argument('candidate', type=Path)
    a = p.parse_args()
    out = {}
    for kind in ('plain', 'chairs', 'dense', 'notched'):
        g = room(kind)
        for start in ((1.5, 3.5), (7.5, 2.0), (4.5, 5.6)):
            key = f'{kind}@{start[0]},{start[1]}'
            out[key] = {name: simulate(load(path, name), g, start)
                        for name, path in (('parent', a.parent), ('candidate', a.candidate))}
    totals = {name: {k: sum(v[name][k] for v in out.values()) for k in ('goals', 'backtracks', 'revisits', 'reversed_headings', 'turn_at_stops_rad', 'path_m')}
              for name in ('parent', 'candidate')}
    print(json.dumps(dict(cases=out, totals=totals), indent=1))
