"""Temporarily prefer discovery outside a repeatedly inspected, camera-observed box."""
import math
from collections import deque


class ClusterExit:
    def __init__(self):
        self.visits = deque(maxlen=64)
        self.observed = ()

    def dispatched(self, goal, elapsed):
        box = goal.get('batch_box')
        if box is not None and goal.get('r32_kind') == 'interior_surface':
            self.visits.append((elapsed, tuple(box)))

    def crowded_centers(self, elapsed):
        centers = []
        for center in self.observed:
            visits = [t for t, point in self.visits if 0 <= elapsed-t <= 150
                      and math.dist(point, center) < .42]
            if len(visits) >= 3 and visits[-1]-visits[0] >= 20:
                centers.append(center)
        return centers

    @staticmethod
    def local_fragment(goal, centers):
        # A fitted, genuinely unobserved face (including on a neighbouring box)
        # keeps its turn. An unowned grid sample is deferred only very near a known fit.
        if 'fitted_face' in goal:
            return False
        owner = goal.get('batch_box')
        point = owner if owner is not None else goal['surface_odom']
        radius = .42 if owner is not None else .78
        return any(math.dist(point, box) < radius for box in centers)
