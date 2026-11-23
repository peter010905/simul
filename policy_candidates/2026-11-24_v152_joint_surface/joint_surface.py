"""Compare safe inspection poses by new camera coverage and route cost."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass
class View:
    goal: dict
    key: tuple
    cell: tuple
    walk: float
    weight: float


def connecting_distances(grid, costmap, safe, nav, start, ends, limit):
    """Bounded grid search through the existing SLAM and Nav2 safety masks."""
    remaining = set(ends)
    distances = {}
    steps_limit = int(limit / grid.resolution)
    queue = deque([(start, 0)])
    visited = {start}
    while queue and remaining:
        cell, steps = queue.popleft()
        if cell in remaining:
            distances[cell] = steps * grid.resolution
            remaining.remove(cell)
        if steps >= steps_limit:
            continue
        row, col = cell
        for nxt in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
            if nxt in visited or not grid.inside(*nxt):
                continue
            visited.add(nxt)
            if not safe[nxt]:
                continue
            nav_cell = costmap.index(*grid.world(*nxt))
            if costmap.inside(*nav_cell) and nav[nav_cell]:
                queue.append((nxt, steps + 1))
    return distances


def inspection_key(goal, wall_key):
    if goal['r32_kind'] == 'wall_surface':
        return ('wall', *wall_key(goal))
    x, y = goal['surface_odom']
    side = round(goal['normal_odom'] / (math.pi / 4)) % 8
    owner = goal.get('batch_box')
    if owner is not None:
        return ('box', round(owner[0] / .5), round(owner[1] / .5), side)
    return ('surface', math.floor(x / .55), math.floor(y / .55), side)


def projected_coverage(grid, view, surfaces, camera, rotation, reference_m, readable_px):
    # Every input is already an unobserved surface with a safe individual view.
    # This prediction contributes to the score only. Camera frames record coverage.
    fx, hfov = camera
    x, y = view.goal['x'], view.goal['y']
    yaw = view.goal['yaw'] + .5 * view.goal.get('scan_angle', 0.)
    coverage = {view.key: view.weight}
    for target in surfaces:
        if target.key in coverage:
            continue
        sx, sy = target.goal['surface_map']
        dx, dy = sx - x, sy - y
        distance = math.hypot(dx, dy)
        if distance < .1 or distance > fx * reference_m / readable_px:
            continue
        bearing = math.atan2(dy, dx)
        if abs(math.remainder(bearing - yaw, math.tau)) > .44 * hfov:
            continue
        normal = target.goal['normal_odom'] - rotation
        alignment = -math.cos(bearing - normal)
        if alignment < .55 or fx * reference_m * alignment / distance < readable_px:
            continue
        if grid.line_clear((x, y), (sx + .10 * math.cos(normal), sy + .10 * math.sin(normal))):
            coverage[target.key] = .70 * target.weight
    return coverage


class JointSurface:
    LOCAL_LEG_M = 2.4
    MAX_GROUP_S = 48.
    MAX_GROUP_GOALS = 3
    TRAVEL_MPS = .20

    def __init__(self):
        self.next_key = None
        self.group_started = 0.
        self.group_count = 0

    def dispatched(self, goal, elapsed):
        if goal.get('joint_views') is None:
            self.next_key = None
            self.group_count = 0
            return
        if not goal['joint_continuation']:
            self.group_started = elapsed
            self.group_count = 0
        self.group_count += 1
        self.next_key = goal['joint_next_key']

    def choose(self, grid, costmap, safe, nav, distances, transform, survey,
               walls, interiors, reserved, elapsed, duration):
        camera = survey.camera
        if camera is None or min(*camera, survey.reference_m, survey.readable_px) <= 0:
            return None
        remaining = duration - elapsed
        views = []
        options = [goal for _, goal in (*walls, *interiors)]
        if reserved is not None:
            options.append(reserved)
        for goal in options:
            cell = grid.index(goal['x'], goal['y'])
            if not grid.inside(*cell) or not safe[cell]:
                continue
            walk = float(distances[cell])
            if not math.isfinite(walk) or walk / self.TRAVEL_MPS + 5 > remaining:
                continue
            key = inspection_key(goal, survey.wall_key)
            if key[0] == 'wall':
                since, last = survey.wall_times.setdefault(key[1:], (elapsed, -math.inf))
                direction_age = elapsed - survey.last_direction.get(key[1], 0.)
                weight = 1. + min(.65, max(elapsed - max(since, last), direction_age) / 150.)
            else:
                weight = 1.4 if key[0] == 'box' else 1.
                if goal.get('completed_box') or goal.get('previous_attempts'):
                    weight *= .75
            views.append(View(goal, key, cell, walk, weight))
        if not views:
            return None
        coverage = [projected_coverage(grid, v, views, camera, transform[2],
                                       survey.reference_m, survey.readable_px) for v in views]
        # Cap incidental observations so splitting a wall into more segments
        # does not overwhelm a directly targeted face.
        def reward(covered):
            return sum(sorted(covered.values(), reverse=True)[:4])

        def score(covered, meters, stops):
            return reward(covered) / (1. + .34 * meters + .28 * (stops - 1))

        singles = [score(c, v.walk, 1) for c, v in zip(coverage, views)]
        # Use the best singles, the previous local continuation, and a
        # representative of each surface type for the bounded two-stop search.
        order = sorted(range(len(views)), key=lambda i: -singles[i])
        continuation = (self.group_count < self.MAX_GROUP_GOALS
                        and elapsed - self.group_started < self.MAX_GROUP_S)
        shortlist = list(dict.fromkeys(order[:6] + [
            i for category in ('wall', 'box', 'surface')
            for i in [next((j for j in order if views[j].key[0] == category), order[0])]
        ] + [next((j for j in order if views[j].key == self.next_key), order[0])]))
        winner = None
        for i in shortlist:
            first = views[i]
            same_group = continuation and first.key == self.next_key and first.walk <= self.LOCAL_LEG_M
            if not continuation and first.key == self.next_key and len(shortlist) > 1:
                continue
            preference = 1.16 if same_group else 1.
            proposals = [(singles[i], first.walk, None, coverage[i])]
            quality = [j for j in shortlist if views[j].key not in coverage[i]
                       and math.dist(first.goal['surface_map'], views[j].goal['surface_map']) <= 3.0]
            if quality:
                legs = connecting_distances(grid, costmap, safe, nav, first.cell,
                                            [views[j].cell for j in quality], self.LOCAL_LEG_M)
                for j in quality:
                    route = first.walk + legs.get(views[j].cell, math.inf)
                    if route / self.TRAVEL_MPS + 10 > remaining:
                        continue
                    union = {**coverage[i], **coverage[j]}
                    for key in coverage[i].keys() & coverage[j].keys():
                        union[key] = max(coverage[i][key], coverage[j][key])
                    proposals.append((score(union, route, 2), route, views[j].key, union))
            for utility, meters, next_key, covered in proposals:
                if winner is None or utility * preference > winner[0]:
                    winner = utility * preference, first, meters, next_key, covered, same_group
        if winner is None:
            return None
        utility, best, meters, next_key, covered, same_group = winner
        goal = dict(best.goal, joint_views=len(covered), joint_walk_m=round(best.walk, 3),
                    joint_route_m=round(meters, 3), joint_score=round(utility, 3),
                    joint_next_key=next_key, joint_continuation=same_group)
        if best.key[0] == 'wall':
            goal['survey_wall'] = list(best.key[1:])
        return goal
