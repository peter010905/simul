"""Allocate wall turns by camera evidence and persistent, direction-specific waiting."""
from collections import defaultdict
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class Section:
    since: float
    last_sent: float = -math.inf


class WallSections:
    def __init__(self):
        self.sections = {}
        self.last_reserved = -math.inf
        self.last_was_reserved = False
        self.period = 45.
        self.axes = None
        self.coverage = {}
        self.served = defaultdict(float)

    def observe(self, samples, grid, viewed, transform):
        # SLAM-axis labels rotate into a fixed odom basis on first use.
        if self.axes is None:
            self.axes = transform[2] + grid.origin[2]
        counts = defaultdict(lambda: [0, 0])
        for (kind, _, side, _, _), cells in samples.items():
            if kind != 'wall':
                continue
            normal = (-math.pi/2, 0., math.pi/2, math.pi)[side]
            direction = round((normal+grid.origin[2]+transform[2]-self.axes)/(math.pi/2)) % 4
            for row, col in cells:
                counts[direction][0] += int(viewed[side,row,col])
                counts[direction][1] += 1
        self.coverage = {key: seen/total for key,(seen,total) in counts.items() if total}

    def key(self, anchor, normal):
        axis = self.axes or 0.
        direction = round((normal-axis)/(math.pi/2)) % 4
        angle = axis+direction*math.pi/2
        nx, ny = math.cos(angle), math.sin(angle)
        x,y = anchor
        return direction, round((nx*x+ny*y)/.50), math.floor((-ny*x+nx*y)/1.20)

    def choose(self, walls, distances, grid, elapsed, duration, has_interior=False):
        self.period = min(45.,max(18.,duration*.075))
        choices = []
        for _,goal in walls:
            key = self.key(goal['surface_odom'],goal['normal_odom'])
            state = self.sections.setdefault(key,Section(elapsed))
            age = elapsed-max(state.since,state.last_sent)
            if age < self.period or elapsed-state.last_sent < 2*self.period:
                continue
            walk = float(distances[grid.index(goal['x'],goal['y'])])
            if not math.isfinite(walk) or walk/.20+6. > duration-elapsed:
                continue
            sector_wait = elapsed-self.served[key[0]]
            debt = 1.-self.coverage.get(key[0],0.)
            # Real camera evidence sets the deficit. Age also serves a neglected
            # direction when another direction repeatedly gains new wall fragments.
            priority = 2.*debt+min(4.,sector_wait/self.period)+min(2.,age/(2*self.period))
            overdue = sector_wait >= 3*self.period
            choices.append((overdue, sector_wait if overdue else priority, -walk, key, goal, age))
        if (not choices or elapsed-self.last_reserved < self.period
                or (has_interior and self.last_was_reserved)):
            return None
        _,_,_,key,goal,age = max(choices,key=lambda item:item[:3])
        return dict(goal,wall_section=list(key),wall_wait_s=round(age,2),
                    wall_direction=key[0],wall_camera_fraction=self.coverage.get(key[0],0.))

    def dispatched(self, goal, elapsed):
        self.last_was_reserved = 'wall_section' in goal
        if self.last_was_reserved:
            self.last_reserved = elapsed
        if goal.get('r32_kind') != 'wall_surface' or 'surface_odom' not in goal:
            return
        key = self.key(goal['surface_odom'],goal['normal_odom'])
        self.sections.setdefault(key,Section(elapsed)).last_sent = elapsed
        self.served[key[0]] = elapsed
