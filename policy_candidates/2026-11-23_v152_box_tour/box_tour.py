"""Plan a bounded visit of the reachable, unobserved faces of the current box."""
from collections import deque
from itertools import permutations
import math


def local_distances(grid, safe, start, endpoints, limit_m):
    """Four-connected traversal of the same clearance mask as dispatch."""
    pending = set(endpoints)
    if not safe[start]:
        return {}
    queue = deque([(start, 0)])
    visited = {start}
    distances = {}
    limit = math.floor(limit_m/grid.resolution)
    while queue and pending:
        cell, steps = queue.popleft()
        if cell in pending:
            pending.remove(cell)
            distances[cell] = steps*grid.resolution
        if steps == limit:
            continue
        row, col = cell
        for neighbor in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if neighbor not in visited and grid.inside(*neighbor) and safe[neighbor]:
                visited.add(neighbor)
                queue.append((neighbor, steps+1))
    return distances


class BoxTour:
    max_visit_s = 75.
    max_leg_m = 4.3
    max_route_m = 8.
    remaining_reserve_s = 55.

    def __init__(self):
        self.clear()

    def clear(self):
        self.center = None
        self.started = None
        self.reference = 0.
        self.visited = set()  # Dispatches only. Camera coverage remains authoritative.
        self.order = ()

    def sector(self, goal):
        return math.floor((goal['normal_odom']-self.reference)/(math.pi/2)+.5) % 4

    def dispatched(self, goal, elapsed):
        center = goal.get('batch_box')
        if center is None:
            if any(goal.get(k) for k in ('survey_region','cluster_exit','survey_escape')):
                self.clear()
            return
        if self.center is None or math.dist(center, self.center) >= .65:
            self.clear()
            self.center = center
            self.started = elapsed
            self.reference = goal['normal_odom']
        sector = self.sector(goal)
        self.visited.add(sector)
        self.order = tuple(i for i in goal.get('box_tour_order',self.order) if i != sector)

    def choose(self, choices, grid, safe, robot_odom, elapsed, duration):
        if (self.center is None or math.dist(robot_odom,self.center) > 2.6
                or elapsed-self.started >= self.max_visit_s or duration-elapsed <= self.remaining_reserve_s):
            return None
        representatives = {}
        for distance, goal in choices:
            center = goal.get('batch_box')
            if (center is None or goal['previous_attempts'] or not 0. <= distance <= self.max_leg_m
                    or math.dist(center,self.center) >= .65
                    or math.dist(goal['surface_odom'],center) > .85):
                continue
            side = self.sector(goal)
            if side in self.visited:
                continue
            quality = ('fitted_face' not in goal, distance)
            existing = representatives.get(side)
            if existing is None or quality < existing[0]:
                representatives[side] = (quality,goal)
        if not representatives:
            return None

        sides = tuple(sorted(representatives))
        cells = {side: grid.index(representatives[side][1]['x'],representatives[side][1]['y'])
                 for side in sides}
        legs = {side: local_distances(grid,safe,cells[side],cells.values(),self.max_leg_m)
                for side in sides} if len(sides) > 1 else {}
        reserved = tuple(side for side in self.order if side in representatives)
        best = None
        for count in range(1,len(sides)+1):
            for order in permutations(sides,count):
                distance = representatives[order[0]][0][1]
                distance += sum(legs[a].get(cells[b],math.inf) for a,b in zip(order,order[1:]))
                estimate_s = distance/.22 + 4.*len(order)
                if (distance > self.max_route_m or estimate_s > duration-elapsed-self.remaining_reserve_s
                        or (len(order)>1 and estimate_s > self.max_visit_s-(elapsed-self.started))):
                    continue
                key = (-len(order), not reserved or order[:len(reserved)] != reserved, distance, order)
                if best is None or key < best[0]:
                    best = (key,order,distance)
        if best is None:
            return None
        _, order, distance = best
        return dict(representatives[order[0]][1], local_batch=True,
                    box_tour_order=list(order), box_tour_route_m=round(distance,3))
