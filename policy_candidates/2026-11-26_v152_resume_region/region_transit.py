"""Keep the destination of an interrupted exploration trip in odometry."""
import math
from collections import deque

from osp_mission.box_views import odom_point
from osp_mission.unvisited_priority import VisitNovelty


class RegionTransit:
    def __init__(self):
        self.anchor = None
        self.active = False
        self.paused = False
        self.expires = 0.
        self.resumes = 0
        self.choices = deque(maxlen=40)
        self.last_escape = -math.inf

    def interrupt(self, reason):
        if reason in ('marker_hint', 'registered_marker', 'readable_marker'):
            if self.active:
                self.paused = True
        else:
            self.paused = False
            self.anchor = None
        self.active = False

    def dispatched(self, goal, now, robot):
        if robot is not None:
            self.choices.append((now, tuple(robot)))
        self.active = 'region_destination_odom' in goal
        if self.active:
            if goal.get('region_action') == 'resume':
                self.resumes += 1
            else:
                self.anchor = tuple(goal['region_destination_odom'])
                self.resumes = 0
                self.expires = now + 90.
            self.paused = False
            if goal.get('region_action') == 'leave':
                self.last_escape = now
        elif not self.paused:
            self.anchor = None

    def mode(self, now, robot, trail):
        if self.paused and self.anchor is not None:
            if now <= self.expires and self.resumes < 2 and math.dist(robot, self.anchor) > .9:
                return 'resume'
            self.paused = False
            self.anchor = None
        if now-self.last_escape < 55.:
            return None
        recent = [(t,p) for t,p in self.choices if 0 <= now-t <= 65.]
        if len(recent) < 4 or now-recent[0][0] < 40.:
            return None
        # Use the driven route between stops as well. A long excursion that
        # happens to end in the same cluster is not continuous confinement.
        if any(math.dist(p, robot) > 2.65 for _, p in recent):
            return None
        if any(math.hypot(x-robot[0], y-robot[1]) > 2.65 for t,x,y in trail
               if recent[0][0] <= t <= now):
            return None
        return 'leave'

    def choose(self, mode, choices, grid, distances, transform, robot, visited,
               remaining, veto):
        if not mode or remaining < 12. or len(visited) < 4:
            return None
        novelty = VisitNovelty(visited)
        best, best_rank = None, None
        for goal in choices:
            if goal.get('stationary') or veto(goal):
                continue
            visual, frontier = goal['features'][:2]
            if 'survey_wall' in goal:
                visual = max(visual, min(2., 1. + .25*(goal.get('survey_visible_sections') or 1)))
            if visual < .35 and frontier < .15:
                continue
            cell = grid.index(goal['x'], goal['y'])
            if not grid.inside(*cell):
                continue
            walk = float(distances[cell])
            if not .65 <= walk <= .20*(remaining-8.):
                continue
            point = odom_point((goal['x'],goal['y']), transform)
            unseen = novelty.distance(goal, transform)
            same_frontier = False
            if mode == 'resume':
                offset = math.dist(point, self.anchor)
                same_frontier = frontier >= .15 and offset <= .25
                if (offset > 1.65 or offset + .55 > math.dist(robot, self.anchor)
                        or (not same_frontier and unseen < .65)):
                    continue
            else:
                offset = 0.
                if unseen < 1.5 or math.dist(robot, point) < 2.7:
                    continue
            value = (min(4., unseen) + min(2., max(visual,frontier))) / (1. + .18*walk + offset)
            # When the old frontier is still available, keep it ahead of
            # higher-scoring substitute viewpoints in the same region.
            rank = (same_frontier, -offset if same_frontier else value)
            if best_rank is None or rank > best_rank:
                best_rank = rank
                best = dict(goal, r32_kind=goal.get('r32_kind','interior_frontier'), survey_region=True,
                            region_action=mode, new_region_distance_m=round(unseen,3),
                            new_region_path_m=round(walk,3))
        return best
