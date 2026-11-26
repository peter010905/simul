"""Prefer unfinished interior surfaces and useful new floor before ordinary wall stops."""
import math
import numpy as np
from scipy.spatial import cKDTree

from osp_mission.unvisited_priority import VisitNovelty


def hinted(goal):
    return goal is not None and any(key in goal for key in
        ('approach_id','focus_id','distant_hint_id','bearing_advance_id','weak_wall','routed_approach_m'))


class InteriorPriority:
    def __init__(self):
        self.metrics = {}

    def choose(self, grid, distances, transform, interiors, frontiers, walls, visited, remaining, veto):
        novelty = VisitNovelty(visited)
        anchors = [g['surface_map'] for _,g in walls if 'surface_map' in g]
        boundary = cKDTree(anchors) if anchors else None
        ranked = []
        for walk, goal in interiors:
            if (goal.get('completed_box') or veto(goal)
                    or not math.isfinite(walk) or walk/.20+8 > remaining):
                continue
            cell = grid.index(goal['x'],goal['y'])
            if not grid.inside(*cell) or not math.isfinite(distances[cell]):
                continue
            known_box = goal.get('batch_box') is not None or goal.get('fitted_face') is not None
            unseen = novelty.distance(goal,transform)
            score = (2.4 if known_box else 1.5)*goal.get('marker_gain',1.)*novelty.new_gain(unseen)/(1+.35*walk+novelty.return_cost(unseen,walk))
            option = dict(goal, interior_priority=True, interior_priority_score=round(score,4))
            ranked.append((score,option))
        frontier_count = 0
        for goal in frontiers:
            if (goal.get('stationary') or goal['features'][1] < .15 or veto(goal)):
                continue
            cell = grid.index(goal['x'],goal['y'])
            if not grid.inside(*cell):
                continue
            walk = float(distances[cell])
            if not math.isfinite(walk) or walk/.20+8 > remaining:
                continue
            # Measured wall surfaces give a soft inward preference. Unknown
            # boundaries and image coordinates never define a fixed room centre.
            wall_gap = float(boundary.query((goal['x'],goal['y']))[0]) if boundary else 0.
            inward = 1.+.30*min(2.,wall_gap)
            score = novelty.frontier_score(goal,transform,walk)*inward
            option = dict(novelty.annotate(goal,transform,walk),r32_kind='interior_frontier',
                          interior_priority=True,interior_priority_score=round(score,4),
                          known_wall_distance_m=round(wall_gap,3))
            ranked.append((score,option))
            frontier_count += 1
        self.metrics = dict(kind='interior_or_new_floor',ranked_choices=len(ranked),frontier_choices=frontier_count)
        return max(ranked,key=lambda item:item[0])[1] if ranked else None
