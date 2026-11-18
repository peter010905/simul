import unittest
import numpy as np

from compare import SquarePriority
from square_face import borders

class SquareFaceTest(unittest.TestCase):
    def test_duplicate_exposure_opposite_faces_and_cooldown(self):
        p = np.array([1., 0.])
        n = np.array([-1., 0.])
        cam = (0., 0., .5)
        m = SquarePriority()
        for stamp, normal in [(1., n),(1., n),(1.2,-n)]:
            m.track_frame([(p, normal, 12.)]*2, stamp, stamp, cam)
        self.assertFalse(m.looks)
        m.track_frame([(p, n, 13.)],1.3,1.3,cam)
        self.assertEqual(m.looks[0]['action'],'turn')
        self.assertAlmostEqual(m.looks[0]['yaw_delta_deg'],-28.65)
        for stamp in [1.4,8.,8.1,16.,16.1]:
            m.track_frame([(p,n,13.)],stamp,stamp,cam)
        self.assertEqual(len(m.looks),1)

    def test_blank_border_and_synthetic_print(self):
        blank = np.full((100,100),240,np.uint8)
        blank[25:75,25:75] = 10
        self.assertFalse(borders(blank))
        for i,j in [(1,1),(2,3),(3,2),(3,4),(4,1),(4,3)]:
            blank[25+i*8:33+i*8,25+j*8:33+j*8] = 245
        self.assertTrue(borders(blank))

    def test_priority_beats_an_unrelated_existing_goal(self):
        from test_surface_memory import SurfaceTest
        fixture = SurfaceTest()
        fixture.setUp()
        m = SquarePriority()
        n = fixture.normal
        weak = [(fixture.point, n, {'too_small'}), (np.array([.25,1.55]),np.array([0.,-1.]),{'too_small'})]
        for t in [0.,.3]:
            m.update(weak,fixture.pose,t)
        for t in [4.,4.1]:
            m.track_frame([(fixture.point,n,20.)],t,t,fixture.pose)
        snap = fixture.synthetic_snapshot()
        snap['candidates'][0,2] = np.pi/2
        ids = m.pending(5.)
        decision=m.choose(snap,fixture.pose,5.,200,ids)
        self.assertEqual((decision['source'],decision['face_id'],decision['kind']),('square',0,'waypoint'))

if __name__=='__main__':
    unittest.main()
