import unittest
import numpy as np
from replay_recovery import borders,plane_hit
from square_face import borders as previous_borders

class RecoveryTest(unittest.TestCase):
    def test_partial_sightings_require_a_confirmation_inside_the_image(self):
        from square_face import SquarePriority
        from recovery import can_confirm
        m=SquarePriority()
        observation=(np.array([1.,0.]),np.array([-1.,0.]),50.)
        for t,u in [(1.,248.),(1.1,247.),(1.2,220.)]:
            m.track_frame([observation],t,t,(0.,0.,0.),
                          confirmable={0} if can_confirm(u,50.,250) else set())
        self.assertEqual([v['t'] for v in m.looks],[1.2])

    def test_visible_evidence_at_crop(self):
        image=np.full((100,100),225,np.uint8)
        image[25:75,62:100]=25
        for y,x in [(35,71),(45,80),(56,72),(56,88)]:
            image[y:y+8,x:x+8]=215
        self.assertEqual(previous_borders(image),[])
        self.assertTrue(borders(image))
        image[25:75,62:100]=25
        self.assertFalse(borders(image))
        self.assertFalse(borders(np.fliplr(image).copy()))

    def test_smooth_plane_and_reject_depth_edge(self):
        class Cal:
            k=[150.,0.,125.]
        samples=[(np.array([1.,y]),np.array([-.75,.66]),set()) for y in np.linspace(-.12,.12,9)]
        p,n=plane_hit(125.,22.,samples,(0.,0.,0.),Cal())
        np.testing.assert_allclose(p,[1.,0.],atol=1e-6)
        np.testing.assert_allclose(n,[-1.,0.],atol=1e-6)
        samples.append((np.array([1.5,0.]),np.array([-1.,0.]),set()))
        self.assertIsNone(plane_hit(125.,22.,samples,(0.,0.,0.),Cal()))

if __name__=='__main__':
    unittest.main()
