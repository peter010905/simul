"""Give persistent, camera-unseen wall sections an occasional normal planning turn."""
from dataclasses import dataclass
import math


@dataclass
class Section:
    since: float
    last_sent: float = -math.inf


class WallSections:
    def __init__(self):
        self.sections = {}
        self.last_reserved = -math.inf
        self.last_was_reserved = False
        self.period = 60.

    @staticmethod
    def key(anchor, normal):
        # Segment numbers stay in odom when SLAM changes the occupancy-grid origin.
        side = round(normal / (math.pi / 4)) % 8
        angle = side * math.pi / 4
        nx, ny = math.cos(angle), math.sin(angle)
        x, y = anchor
        return side, round((nx*x + ny*y) / .50), math.floor((-ny*x + nx*y) / 1.20)

    def choose(self, walls, distances, grid, elapsed, duration, has_interior=False):
        self.period = min(60., max(18., duration * .10))
        choices = []
        for _, goal in walls:
            key = self.key(goal['surface_odom'], goal['normal_odom'])
            state = self.sections.setdefault(key, Section(elapsed))
            # A goal is supplied only when the parent finds a camera-unseen face
            # and an admissible viewpoint. Missing sectors cannot become targets.
            age = elapsed - max(state.since, state.last_sent)
            if age < self.period or elapsed - state.last_sent < 2*self.period:
                continue
            walk = float(distances[grid.index(goal['x'], goal['y'])])
            observation_s = 5.
            travel_s = walk / .20
            if not math.isfinite(walk) or travel_s + observation_s > duration-elapsed:
                continue
            # Travel competes with age inside one service interval. Beyond it,
            # a neglected sector eventually outranks a nearby recent sector.
            priority = age - min(self.period, travel_s)
            choices.append((priority, -walk, key, goal, age))
        if not choices or elapsed-self.last_reserved < self.period or (has_interior and self.last_was_reserved):
            return None
        _, _, key, goal, age = max(choices, key=lambda item: item[:2])
        return dict(goal, wall_section=list(key), wall_wait_s=round(age, 2))

    def dispatched(self, goal, elapsed):
        self.last_was_reserved = 'wall_section' in goal
        if self.last_was_reserved:
            self.last_reserved = elapsed
        if goal.get('r32_kind') != 'wall_surface' or 'surface_odom' not in goal:
            return
        key = self.key(goal['surface_odom'], goal['normal_odom'])
        self.sections.setdefault(key, Section(elapsed)).last_sent = elapsed
