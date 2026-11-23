"""Bound local obstacle inspections and return to the previous wall route."""
import math


class WallFirst:
    def __init__(self):
        self.object_center = None
        self.started = None
        self.visits = 0
        self.deferred = []
        self.probe_wall = False

    def obstacle(self, interiors, robot, elapsed, remaining):
        """Choose a fresh, already reachable view of one nearby obstacle."""
        for walk, goal in interiors:
            center = goal.get('batch_box') or goal.get('patrol_object')
            if center is None or goal.get('completed_box') or goal['previous_attempts']:
                continue
            if walk > 3.5 or math.dist(robot, center) > 2.3 or walk / .20 + 4 > remaining:
                continue
            if self.object_center is not None:
                if elapsed - self.started >= 45 or self.visits >= 4:
                    return None
                if math.dist(center, self.object_center) > .4:
                    continue
            elif any(elapsed < until and math.dist(center, point) <= .4
                     for point, until in self.deferred):
                continue
            return dict(goal, patrol_inspection=list(center))
        return None

    def dispatched(self, goal, elapsed):
        # A marker visit may preempt either task without losing its place.
        if any(key in goal for key in ('approach_id', 'focus_id', 'distant_hint_id', 'print_point')):
            self.probe_wall = False
            return
        center = goal.get('patrol_inspection')
        if center is not None:
            if self.object_center is None:
                self.object_center, self.started, self.visits = center, elapsed, 0
            self.visits += 1
            self.probe_wall = False
            return
        returning = self.object_center is not None
        if returning:
            self.deferred = [(p, t) for p, t in self.deferred if elapsed < t]
            self.deferred.append((self.object_center, elapsed + 75.))
            self.object_center = None
        # Complete a wall leg after a detour before allowing another interruption.
        self.probe_wall = goal.get('r32_kind') == 'wall_surface' and not returning
