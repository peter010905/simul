"""Select useful viewpoints in regions the robot has yet to traverse."""
import math

import numpy as np

from osp_mission.box_views import odom_point


def choose_unvisited(choices, grid, cost, nav, safe, distances, transform,
                     trail, elapsed, remaining, veto):
    recent = np.asarray([p[1:] for p in trail if 0 <= elapsed-p[0] <= 200.])
    if len(recent) < 4:
        return None
    best = None
    for candidate in choices:
        surfaces, frontier = candidate['features'][:2]
        if candidate.get('stationary') or (surfaces < .35 and frontier < .15) or veto(candidate):
            continue
        point = candidate['x'], candidate['y']
        cell, nav_cell = grid.index(*point), cost.index(*point)
        if (not grid.inside(*cell) or not safe[cell] or
                not cost.inside(*nav_cell) or not nav[nav_cell]):
            continue
        walk = float(distances[cell])
        if not math.isfinite(walk) or walk > .2*(remaining-8):
            continue
        position = odom_point(point, transform)
        novelty = float(np.linalg.norm(recent-position, axis=1).min())
        if novelty < 1.5:
            continue
        rank = (.65 * min(surfaces, 2.) + .45 * min(frontier, 2.) + .65 * min(novelty, 6.)) / (1. + .13 * walk)
        if best is None or rank > best[0]:
            best = rank, dict(candidate, r32_kind='interior_frontier', survey_region=True,
                              new_region_distance_m=round(novelty, 3), new_region_path_m=round(walk, 3))
    return best[1] if best else None
