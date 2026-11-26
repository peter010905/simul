"""Rank useful camera stops and frontiers by how new their destination is."""
from scipy.spatial import cKDTree

from osp_mission.box_views import odom_point


class VisitNovelty:
    def __init__(self, visited):
        self.tree = cKDTree(visited) if len(visited) >= 4 else None

    def distance(self, goal, transform):
        if self.tree is None:
            return None
        return float(self.tree.query(odom_point((goal['x'], goal['y']), transform))[0])

    @staticmethod
    def new_gain(distance):
        if distance is None:
            return 1.
        return 1. + .85*min(1., max(0., (distance-.65)/2.35))

    @staticmethod
    def return_cost(distance, walk):
        if distance is None:
            return 0.
        near_old_track = max(0., 1.-distance/1.35)
        return .95*near_old_track*min(1., max(0., (walk-.45)/1.15))

    def frontier_score(self, goal, transform, walk):
        visual, unknown = goal['features'][:2]
        distance = self.distance(goal, transform)
        gain = min(2.25, 1.15 + max(0., unknown) + .25*max(0., visual))
        return (gain*self.new_gain(distance)
                / (1. + .45*walk + self.return_cost(distance, walk)))

    def annotate(self, goal, transform, walk):
        distance = self.distance(goal, transform)
        return dict(goal, visit_novelty_m=None if distance is None else round(distance, 3),
                    visit_novelty_gain=round(self.new_gain(distance), 3),
                    visit_return_cost=round(self.return_cost(distance, walk), 3))
