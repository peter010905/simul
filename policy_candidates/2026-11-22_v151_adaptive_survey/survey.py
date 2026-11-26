"""Allocate safe observation goals to camera-unseen surfaces and spatial discovery."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from osp_mission.edge_policy import readable_distance


def odom_xy(point, transform):
    x, y, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Option:
    kind: str
    goal: dict
    walk: float
    turn: float
    seconds: float
    repeat: float
    region: tuple


class AdaptiveSurvey:
    """The task owner retains Nav2, action lifecycle, hazards and marker interrupts."""

    def __init__(self):
        self.camera = None
        self.waiting = {}
        self.routes = deque(maxlen=96)
        self.regions = {}

    def samples(self, grid, viewed, groups):
        # Represent each existing bin by a genuinely camera-unseen boundary cell.
        points, normals, length, interior = [], [], [], []
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        for (kind, _, side, _, _), cells in groups.items():
            unseen = [(r, col) for r, col in cells if not viewed[side, r, col]]
            if len(unseen) * grid.resolution < .15:
                continue
            r, col = unseen[len(unseen) // 2]
            dy, dx = ((-1, 0), (0, 1), (1, 0), (0, -1))[side]
            points.append(grid.world(r + .5 * dy, col + .5 * dx))
            normals.append((c * dx - s * dy, s * dx + c * dy))
            length.append(len(unseen) * grid.resolution)
            interior.append(kind == 'interior')
        return (np.asarray(points, dtype=float).reshape(-1, 2),
                np.asarray(normals, dtype=float).reshape(-1, 2),
                np.asarray(length), np.asarray(interior, dtype=bool))

    def gains(self, goal, grid, surfaces, read_distance, fov):
        points, normals, lengths, interior = surfaces
        origin = np.array([goal['x'], goal['y']])
        delta = origin - points
        distance = np.linalg.norm(delta, axis=1)
        incidence = np.sum(delta * normals, axis=1) / np.maximum(distance, 1.e-6)
        scan = goal.get('scan_angle', 0.)
        center = goal['yaw'] + scan / 2.
        angles = np.arctan2(-delta[:, 1], -delta[:, 0]) - center
        relative = np.abs(np.arctan2(np.sin(angles), np.cos(angles)))
        # Predict read-quality opportunity using the deployed minimum-size prior;
        # only the exposure callback can mark a cell observed.
        visible = ((distance > .25) & (incidence > .6) &
                   (distance < read_distance * incidence) &
                   (relative < min(math.pi, (fov + abs(scan)) / 2.)))
        obstacle = wall = 0.
        for i in np.flatnonzero(visible):
            if grid.line_clear(origin, points[i] + .06 * normals[i]):
                if interior[i]:
                    obstacle += lengths[i]
                else:
                    wall += lengths[i]
        return wall, obstacle

    def option(self, kind, goal, grid, costmap, nav, distances, clearance, pose,
               transform, elapsed, remaining, veto):
        if goal.get('stationary') or veto(goal):
            return None
        point = goal['x'], goal['y']
        cell = grid.index(*point)
        nc = costmap.index(*point)
        if (not grid.inside(*cell) or not costmap.inside(*nc) or
                not nav[nc] or clearance[cell] < .32):
            return None
        walk = float(distances[cell])
        turn = abs(wrapped(goal['yaw'] - pose[2]))
        # This is the existing four-connected grid travel proxy in metres.
        # Nav2 owns actual path planning and collision checking at dispatch.
        seconds = walk / .22 + (turn + abs(goal.get('scan_angle', 0.))) / .5 + 2.
        if not math.isfinite(seconds) or seconds > remaining:
            return None
        region = tuple(goal.get('surface_odom', odom_xy(point, transform)))
        repeat = sum(max(0., 1. - (elapsed - t) / 160.) *
                     max(0., 1. - math.dist(region, other) / 2.4)
                     for t, other in self.routes if 0 <= elapsed - t < 160.)
        return Option(kind, goal, walk, turn, seconds, repeat, region)

    def choose(self, schedule, grid, costmap, nav, distances, clearance, pose,
               transform, viewed, samples, candidates, choices, elapsed, duration, veto):
        if not self.camera:
            return None
        focal, fov = self.camera
        if not (math.isfinite(focal) and focal > 0 and 0 < fov < math.pi):
            return None
        # Match edge_policy.readable_distance. Never expand the covered area
        # on the strength of a far-away silhouette alone.
        read = readable_distance(focal)
        remaining = duration - elapsed
        if remaining <= 2.:
            return None
        eligible = {}
        crowded = schedule.cluster.crowded_centers(elapsed)
        frontiers = [g for g in choices if g['features'][1] >= .15
                     and not any(k in g for k in ('approach_id', 'focus_id', 'print_odom',
                                                   'surface_odom', 'adaptive_survey'))]
        grouped = {'wall': [g for _, g in candidates['wall']],
                   'interior': [g for _, g in candidates['interior']
                                if not g.get('completed_box')],
                   'frontier': frontiers}
        for kind, goals in grouped.items():
            eligible[kind] = []
            for g in goals:
                opt = self.option(kind, g, grid, costmap, nav, distances, clearance,
                                  pose, transform, elapsed, remaining, veto)
                if opt is not None:
                    eligible[kind].append(opt)
            if eligible[kind]:
                self.waiting.setdefault(kind, elapsed)
            else:
                # Time with no feasible, affordable goal is not service debt.
                self.waiting.pop(kind, None)
        # A region accrues age only while it has at least one feasible viewpoint.
        # Memory scales with the currently reachable map, not mission frame count.
        active_regions = {self.region_key(option) for options in eligible.values() for option in options}
        self.regions = {key: self.regions.get(key, elapsed) for key in active_regions}
        if not any(eligible.values()):
            return None

        weak = schedule.weak.choose(candidates['wall'], grid, transform, schedule.weak_known,
                                    schedule.weak_clock, remaining)
        if weak is not None:
            cue = self.option('wall', weak, grid, costmap, nav, distances, clearance,
                              pose, transform, elapsed, remaining, veto)
            if cue is not None:
                return self.result(cue, 'persistent_small_wall_candidate', 0., 0., 0., elapsed)

        surfaces = self.samples(grid, viewed, samples)
        period = min(90., max(20., .15 * duration))
        ranked = []
        for kind, options in eligible.items():
            if not options:
                continue
            wait = max(0., elapsed - self.waiting[kind])
            # Remove near-duplicate wall fragments/poses before the expensive LOS
            # score. Distinct normals stay eligible in the same small area.
            def region_debt(option):
                age = max(0., elapsed - self.regions[self.region_key(option)])
                return int(age // (2. * period))

            ordered = sorted(options, key=lambda o: (-region_debt(o), self.shortlist_cost(o, crowded, schedule)))
            slots = set()
            shortlist = []
            for option in ordered:
                x, y = option.region
                orientation = option.goal.get('normal_odom', option.goal['yaw'] + transform[2])
                slot = (math.floor(x / 1.), math.floor(y / 1.),
                        round(orientation / (math.pi / 2)) % 4)
                if slot in slots:
                    continue
                slots.add(slot)
                shortlist.append(option)
                if len(shortlist) >= 12:
                    break
            best_in_class = None
            for option in shortlist:
                variants = [option]
                if kind == 'wall' and not option.goal.get('reopened_face'):
                    # The baseline prefers .67m, often spending one turn per
                    # wall fragment. Widen the view while retaining resolution,
                    # LOS, known floor, live costmap connectivity and veto.
                    goal = option.goal
                    normal_yaw = goal['normal_odom'] - transform[2]
                    normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
                    face = np.asarray(goal['surface_map'])
                    stand = min(1.8, .86 * read)
                    point = face + normal * stand
                    if stand > .7 and math.dist(point, (goal['x'], goal['y'])) > .15:
                        yaw = math.atan2(-normal[1], -normal[0]) - goal['scan_angle'] / 2.
                        alternate = dict(goal, x=float(point[0]), y=float(point[1]), yaw=yaw,
                                         distance=math.dist(point, pose[:2]), standoff_m=stand)
                        other = self.option(kind, alternate, grid, costmap, nav, distances, clearance,
                                            pose, transform, elapsed, remaining, veto)
                        if other is not None and grid.line_clear(point, face + .06 * normal):
                            variants.append(other)
                for view in variants:
                    wall, interior = self.gains(view.goal, grid, surfaces, read, fov)
                    gain = min(2., wall) + min(2., interior)
                    frontier = float(view.goal['features'][1]) if kind == 'frontier' else 0.
                    value = 1. + gain + min(1.5, frontier)
                    if kind == 'interior':
                        if view.goal.get('fitted_face') is not None:
                            value += .65
                        if view.goal.get('local_batch'):
                            value += .45
                        if schedule.cluster.local_fragment(view.goal, crowded):
                            value *= .45
                    # A bounded travel/overlap cost allows low-density parts of a
                    # room to compete, with an overdue slot for each eligible class.
                    score = value / (1. + view.seconds / 24. + view.repeat) + min(1., wait / period)
                    record = (region_debt(view), score, -view.seconds, view, wall, interior, frontier)
                    if best_in_class is None or record[:3] > best_in_class[:3]:
                        best_in_class = record
            if best_in_class is not None:
                ranked.append((wait if wait >= period else -1., *best_in_class[1:]))
        if not ranked:
            return None
        debt, score, _, chosen, wall, interior, frontier = max(ranked, key=lambda item: item[:3])
        reason = 'overdue_camera_or_frontier' if debt >= 0. else 'unseen_gain_per_travel'
        return self.result(chosen, reason, wall, interior, frontier, elapsed)

    @staticmethod
    def region_key(view):
        x, y = view.region
        return view.kind, math.floor(x / 2.4), math.floor(y / 2.4)

    @staticmethod
    def shortlist_cost(view, crowded, schedule):
        if view.kind == 'frontier':
            return (view.seconds + 12. * view.repeat) / (1. + min(1.5, view.goal['features'][1]))
        penalty = 18. if view.kind == 'interior' and schedule.cluster.local_fragment(view.goal, crowded) else 0.
        return view.seconds + 12. * view.repeat + penalty - 7. * bool(view.goal.get('local_batch'))

    def result(self, option, reason, wall, interior, frontier, elapsed):
        goal = dict(option.goal, adaptive_survey=True, survey_kind=option.kind,
                    survey_region=list(option.region), survey_travel_m=option.walk,
                    survey_region_wait_s=max(0., elapsed - self.regions.get(self.region_key(option), elapsed)),
                    survey_expected_s=option.seconds, survey_wall_gain_m=wall,
                    survey_interior_gain_m=interior, survey_frontier_gain=frontier,
                    survey_wait_s=max(0., elapsed - self.waiting.get(option.kind, elapsed)))
        if option.kind == 'frontier':
            goal['r32_kind'] = 'interior_frontier'
        return ('wall' if option.kind == 'wall' else 'interior', reason, goal)

    def dispatched(self, goal, elapsed, phase):
        kind = goal.get('survey_kind')
        if kind is not None:
            self.waiting[kind] = elapsed
            region = tuple(goal['survey_region'])
            self.routes.append((elapsed, region))
            self.regions[kind, math.floor(region[0] / 2.4), math.floor(region[1] / 2.4)] = elapsed
