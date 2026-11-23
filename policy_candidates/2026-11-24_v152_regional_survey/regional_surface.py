"""Spend a bounded visit surveying nearby surfaces before changing regions."""
import math

from osp_mission.joint_surface import View, inspection_key, projected_coverage


def region_key(point):
    return tuple(math.floor(coordinate / 2.) for coordinate in point)


class RegionalSurface:
    def __init__(self):
        self.anchor = None
        self.entered = 0.
        self.stops = 0
        self.attempted = set()
        self.recent_regions = {}

    def dispatched(self, goal, elapsed):
        if 'region_anchor' not in goal:
            # A marker interruption can resume the same region within its
            # original budget. Recovery does not create extra inspection time.
            return
        if not goal['joint_continuation']:
            if self.anchor is not None:
                self.recent_regions[region_key(self.anchor)] = elapsed
            self.anchor = tuple(goal['region_anchor'])
            self.entered = elapsed
            self.stops = 0
            self.attempted.clear()
        self.stops += 1
        self.attempted.add(goal['region_face'])

    def can_continue(self, elapsed):
        return self.anchor is not None and self.stops < 4 and elapsed - self.entered < 55.

    def choose(self, grid, safe, distances, transform, survey,
               walls, interiors, reserved, elapsed, duration, *, frontier=None):
        if survey.camera is None or any(v <= 0 for v in (*survey.camera, survey.reference_m, survey.readable_px)):
            return None
        remaining = duration - elapsed
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
            views.append(View(goal, key, cell, walk, weight))
        if not views:
            return None

        active = self.can_continue(elapsed)
        ranked = []
        for view in views:
            point = view.goal['surface_odom']
            same_area = self.anchor is not None and math.dist(point, self.anchor) <= 2.2
            if same_area and view.key in self.attempted:
                continue
            # Geometry of the predicted view can combine wall and obstacle
            # rewards, while the existing camera coverage owns completion.
            seen = projected_coverage(grid, view, views, survey.camera, transform[2],
                                      survey.reference_m, survey.readable_px)
            gain = sum(sorted(seen.values(), reverse=True)[:3])
            visit_age = elapsed - self.recent_regions.get(region_key(point), -math.inf)
            repeat_cost = max(0., 1. - visit_age / 110.)
            if same_area and not active:
                repeat_cost += .8
            score = gain / (1. + .45 * view.walk + repeat_cost)
            ranked.append((score, view, seen, same_area))
        if not ranked:
            return None
        ranked.sort(key=lambda p: -p[0])
        score, selected, seen, _ = ranked[0]
        local = [p for p in ranked if active and p[3] and p[1].walk <= 2.7]
        continuing = bool(local and local[0][0] >= .65 * score)
        if continuing:
            score, selected, seen, _ = local[0]
        elif frontier is not None:
            # Frontier gain uses a different unit than camera coverage. Only
            # promote a reachable, informative frontier once the current area
            # has no worthwhile continuation and the surface is a repeat/long leg.
            target = grid.index(frontier['x'], frontier['y'])
            if grid.inside(*target) and safe[target]:
                walk = float(distances[target])
                gain = float(frontier['features'][1])
                frontier_score = min(2., 1.15 + gain) / (1. + .45 * walk)
                if (0 <= walk <= .20 * (remaining - 8) and frontier_score > 1.15 * score
                        and (selected.walk > 2.3 or ranked[0][3])):
                    return frontier
        goal = dict(selected.goal, joint_views=len(seen), joint_walk_m=round(selected.walk, 3),
                    joint_score=round(score, 3), joint_continuation=continuing,
                    region_anchor=self.anchor if continuing else selected.goal['surface_odom'],
                    region_face=selected.key)
        if selected.key[0] == 'wall':
            goal['survey_wall'] = list(selected.key[1:])
        return goal
