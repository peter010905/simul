"""Order up to four box-face views by connected free-space travel, including table legs."""
import itertools
import math

from osp_mission._r32_navigation_core import travel_distances


def short_tour(options, grid, safe, time_left):
    groups = []
    for walk, goal in sorted(options, key=lambda row: row[0]):
        if any(math.cos(goal['normal_odom']-g['normal_odom']) > .70 for _,g in groups):
            continue
        groups.append((walk,goal))
        if len(groups) == 4:
            break
    if not groups:
        return None
    # The mask already intersects raw SLAM clearance, global Nav2 connectivity
    # and the fresh rolling local costmap. Table legs remain occupied.
    cells = [grid.index(g['x'],g['y']) for _,g in groups]
    pair = {}
    for i, cell in enumerate(cells):
        if not grid.inside(*cell) or not safe[cell]:
            continue
        distances = travel_distances(safe, *cell, grid.resolution)
        for j, other in enumerate(cells):
            pair[i,j] = float(distances[other])
    best = None
    for n in range(1,len(groups)+1):
        for order in itertools.permutations(range(len(groups)),n):
            walk = groups[order[0]][0] + sum(pair.get((a,b),math.inf) for a,b in zip(order,order[1:]))
            if walk/.20 + 8*n > time_left:
                continue
            rank = (-n, walk)
            if best is None or rank < best[0]:
                best = rank, order, walk
    if best is None:
        return None
    _, order, walk = best
    return dict(groups[order[0]][1], box_tour_faces=len(order), box_tour_walk_m=round(walk,3))
