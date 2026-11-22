"""Prioritize safe unobserved faces using only onboard tracks and verified marker positions."""
import math


class BoxBatch:
    def __init__(self):
        self.centers = []
        self.completed = set()
        self.active = None
        self.robot = None

    def update(self, centers, verified, robot):
        self.centers = [tuple(p) for p in centers]
        self.robot = robot
        self.completed = {box for p in verified if (box := self.owner(p)) is not None}

    def owner(self, point):
        near = sorted((math.dist(point, c), c) for c in self.centers if math.dist(point, c) <= .75)
        if not near or (len(near) > 1 and near[1][0] - near[0][0] < .20):
            return None
        return near[0][1]

    def rank(self, view):
        distance, goal = view
        box = self.owner(goal['surface_odom'])
        if box is None and 'generic_object_center_odom' in goal:
            box = tuple(goal['generic_object_center_odom'])
        near = box is not None and math.dist(self.robot, box) <= 2.3
        same = box is not None and self.active is not None and math.dist(box, self.active) < .65
        # Keep a local inspection together only while another face is fresh and reachable.
        batch = near and same and not goal['previous_attempts'] and distance <= 3.5
        goal['batch_box'] = box
        goal['completed_box'] = box is not None and box in self.completed
        goal['local_batch'] = batch
        return (goal['completed_box'], not batch, 'fitted_face' not in goal, distance)

    def dispatched(self, goal):
        # Looking at a nearby wall does not erase the unfinished box batch.
        # A deliberate move to another region does end that local trip.
        if 'batch_box' in goal:
            self.active = goal['batch_box']
        elif goal.get('survey_region') or goal.get('cluster_exit') or goal.get('survey_escape'):
            self.active = None
