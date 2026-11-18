import math
import unittest

import numpy as np

from surface_memory import SurfaceMemory


class SurfaceTest(unittest.TestCase):
    def setUp(self):
        self.memory = SurfaceMemory()
        self.point = np.array([1.55, 1.05])
        self.normal = np.array([-1., 0.])
        self.reason = {'too_small'}
        self.pose = (0., 0., 0.)
        weak = (self.point, self.normal, self.reason)
        self.memory.update([weak, weak], self.pose, 0.)
        self.assertIsNone(self.memory.faces[0].queued)
        self.memory.update([weak], self.pose, .5)

    def test_observing_only_one_side(self):
        reverse = (self.point, -self.normal, set())
        self.memory.update([reverse], self.pose, 10.)
        self.memory.update([reverse], self.pose, 10.3)
        self.assertEqual(self.memory.pending(11.), [0])
        self.assertEqual(len(self.memory.faces), 2)
        good = (self.point, self.normal, set())
        self.memory.update([good], (.5, 0, 0), 11.)
        self.memory.update([good], (.5, 0, 0), 11.4)
        self.assertEqual(self.memory.pending(12.), [])
        self.assertTrue(self.memory.faces[0].revisited)

    def synthetic_snapshot(self):
        grid = np.zeros((40, 40), dtype='int8')
        grid[:, 15] = 100
        return dict(candidates=np.array([[.25,1.05,0,0,0,0,2.,0,0],
                                         [1.05,1.05,0,0,0,0,2.8,0,0]]),
                    selected_index=0, robot=np.zeros(2), heading=0.,
                    origin=np.zeros(2), resolution=.1, data=grid)

    def test_reachable_candidate_and_cooldown(self):
        snap = self.synthetic_snapshot()
        choose = lambda t, ids: self.memory.choose(snap, self.pose, t, 200, ids)
        self.assertIsNone(choose(8., []))
        snap['candidates'][1, 6] = 3.1
        self.assertIsNone(choose(8., [0]))
        snap['candidates'][1, 6] = 2.8
        snap['data'][:, 12] = 100
        self.assertIsNone(choose(8., [0]))
        snap['data'][:, 12] = 0
        self.assertEqual(choose(8., [0])['kind'], 'waypoint')
        self.assertIsNone(choose(60., [0]))
        self.assertEqual(choose(100., [0])['kind'], 'waypoint')

    def test_keep_existing_view_or_turn_in_place(self):
        snap = self.synthetic_snapshot()
        snap['selected_index'] = 1
        self.assertIsNone(self.memory.choose(snap, self.pose, 8., 200, [0]))
        snap['candidates'][1, 2] = math.pi
        proposal = self.memory.choose(snap, self.pose, 8., 200, [0])
        self.assertEqual(proposal['kind'], 'turn')
        self.assertEqual(proposal['extra_path_m'], 0.)
        self.assertAlmostEqual(proposal['goal_yaw'], 0.)


if __name__ == '__main__':
    unittest.main()
