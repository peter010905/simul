"""Spend a bounded visit surveying nearby surfaces before changing regions."""
import math

from osp_mission.joint_surface import View, inspection_key, projected_coverage
from osp_mission.unvisited_priority import VisitNovelty


def region_key(point):
    return tuple(math.floor(coordinate / 2.) for coordinate in point)


class RegionalSurface:
    def __init__(self):
        self.anchor = None
        self.entered = None
        self.robot_xy = None
        self.stops = 0
        self.attempted = set()
        self.recent_regions = {}

    def position(self, x, y, elapsed):
        if not all(math.isfinite(value) for value in (x, y, elapsed)):
            return
        self.robot_xy = x, y
        if self.anchor is not None and self.entered is None and math.dist(self.robot_xy, self.anchor) <= 2.2:
            self.entered = elapsed

    def dispatched(self, goal, elapsed):
        if goal.get('r32_kind') == 'interior_frontier' and self.anchor is not None:
            self.recent_regions[region_key(self.anchor)] = elapsed
            self.anchor = None
        if 'region_anchor' not in goal:
            # A marker interruption can resume the same region within its
            # original budget. Recovery does not create extra inspection time.
            return
        # A new face in the same area cannot reset a visit's stop/time budget.
        if self.anchor is None or math.dist(self.anchor, goal['region_anchor']) > 2.2:
            if self.anchor is not None:
                self.recent_regions[region_key(self.anchor)] = elapsed
            self.anchor = tuple(goal['region_anchor'])
            self.entered = None
            if self.robot_xy is not None:
                self.position(*self.robot_xy, elapsed)
            else:
                # Retain bounded behavior when no odometry sample has arrived.
                self.entered = elapsed
            self.stops = 0
            self.attempted.clear()
        self.stops += 1
        self.attempted.add(goal['region_face'])

    def can_continue(self, elapsed):
        return (self.anchor is not None and self.entered is not None and
                self.stops < 4 and 0 <= elapsed - self.entered < 55.)

    def choose(self, grid, safe, distances, transform, survey,
               walls, interiors, reserved, elapsed, duration, *, frontier=None, heading=None, new_region=None, allow_wall_destinations=True):
        if survey.camera is None or any(v <= 0 for v in (*survey.camera, survey.reference_m, survey.readable_px)):
            return None
        remaining = duration - elapsed
        novelty = VisitNovelty(survey.visited_positions(elapsed))
        feasible_frontiers = []
        for candidate in (frontier, new_region):
            if candidate is None:
                continue
            cell = grid.index(candidate['x'], candidate['y'])
            if grid.inside(*cell) and safe[cell] and 0 <= distances[cell] <= .20*(remaining-8):
                walk = float(distances[cell])
                feasible_frontiers.append((novelty.frontier_score(candidate, transform, walk),
                                           novelty.annotate(candidate, transform, walk)))
        best_frontier = max(feasible_frontiers, key=lambda item: item[0], default=None)
        options = [goal for _, goal in (*walls, *interiors)]
        if reserved is not None:
            options.append(reserved)
        views = []
        for goal in options:
            cell = grid.index(goal['x'], goal['y'])
            if not grid.inside(*cell) or not safe[cell]:
                continue
            walk = float(distances[cell])
            if not 0 <= walk <= .20 * (remaining - 6):
                continue
            key = inspection_key(goal, survey.wall_key)
            retry = goal.get('completed_box') or goal.get('previous_attempts')
            if key[0] == 'wall':
                age = elapsed - survey.last_direction.get(key[1], 0.)
                weight = 1. + min(.35, age / 180.)
            else:
                weight = (1.55 if key[0] == 'box' else 1.20) * (.75 if retry else 1.)
            views.append(View(goal, key, cell, walk, weight * goal.get('marker_gain', 1.)))
        if not views:
            return best_frontier[1] if best_frontier is not None else None

        active = self.can_continue(elapsed)
        # Only the existing wall reservation chooses a deliberate wall stop
        # when there is reachable new object work or a feasible frontier.
        # All visible walls still count as incidental gain on an object route.
        alternate_work = any(v.key[0] != 'wall' and not v.goal.get('previous_attempts')
                             and not v.goal.get('completed_box') and not (self.anchor is not None
                             and math.dist(v.goal['surface_odom'], self.anchor) <= 2.2
                             and v.key in self.attempted) for v in views)
        if frontier is not None:
            cell = grid.index(frontier['x'], frontier['y'])
            alternate_work |= bool(grid.inside(*cell) and safe[cell] and
                                   0 <= distances[cell] <= .20 * (remaining - 8))
        ranked = []
        for view in views:
            if not allow_wall_destinations and view.key[0] == 'wall':
                continue
            if alternate_work and view.key[0] == 'wall' and view.goal is not reserved:
                continue
            point = view.goal['surface_odom']
            same_area = self.anchor is not None and math.dist(point, self.anchor) <= 2.2
            if same_area and view.key in self.attempted:
                continue
            # Geometry of the predicted view can combine wall and obstacle
            # rewards, while the existing camera coverage owns completion.
            seen = projected_coverage(grid, view, views, survey.camera, transform[2],
                                      survey.reference_m, survey.readable_px)
            extra_wall = max((v for k, v in seen.items() if k[0] == 'wall' and k != view.key), default=0.)
            extra_object = max((v for k, v in seen.items() if k[0] != 'wall' and k != view.key), default=0.)
            gain = view.weight + .35 * (extra_wall + extra_object)
            visit_age = elapsed - self.recent_regions.get(region_key(point), -math.inf)
            repeat_cost = max(0., 1. - visit_age / 110.)
            if same_area and not active:
                repeat_cost += .8
            turn = abs(math.remainder(view.goal['yaw']-heading, math.tau)) if heading is not None else 0.
            distance = novelty.distance(view.goal, transform)
            follow_box = (active and same_area and view.goal.get('local_batch')
                          and not view.goal.get('completed_box') and not view.goal.get('previous_attempts'))
            if not follow_box:
                gain *= novelty.new_gain(distance)
                repeat_cost += novelty.return_cost(distance, view.walk)
            score = gain / (1. + .45 * view.walk + .10 * turn + repeat_cost)
            ranked.append((score, view, seen, same_area))
        if not ranked:
            return best_frontier[1] if best_frontier is not None else None
        ranked.sort(key=lambda p: -p[0])
        score, selected, seen, _ = ranked[0]
        local = []
        if active:
            # Known free routes around the same obstacle may exceed its direct radius.
            # Reserve time for the actual inspection before accepting a larger detour.
            time_left = min(duration - elapsed, 55. - (elapsed - self.entered))
            for row in ranked:
                _, view, _, same_area = row
                allowance = 2.7
                if view.key[0] != 'wall' and not view.goal.get('previous_attempts'):
                    allowance = min(4.4, max(0., .20 * (time_left - 8.)))
                if same_area and view.walk <= allowance:
                    local.append(row)
        same_box = [row for row in local if row[1].goal.get('local_batch')
                    and not row[1].goal.get('completed_box') and not row[1].goal.get('previous_attempts')]
        if same_box:
            local = same_box
        competing_score = max(score, best_frontier[0]) if best_frontier else score
        continuing = bool(local and (same_box or local[0][0] >= .78 * competing_score))
        if continuing:
            score, selected, seen, _ = local[0]
        elif best_frontier is not None and best_frontier[0] > score:
            return best_frontier[1]
        selected_goal = novelty.annotate(selected.goal, transform, selected.walk)
        goal = dict(selected_goal, joint_views=len(seen), joint_walk_m=round(selected.walk, 3),
                    joint_score=round(score, 3), joint_continuation=continuing,
                    region_anchor=self.anchor if continuing else selected.goal['surface_odom'],
                    region_face=selected.key)
        if selected.key[0] == 'wall':
            goal['survey_wall'] = list(selected.key[1:])
        return goal
