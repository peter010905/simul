import unittest
from types import SimpleNamespace
import numpy as np
from far_face import far_face

class FarFaceTest(unittest.TestCase):
    def setUp(self):
        angles=np.linspace(-.1,.1,41)
        self.scan=SimpleNamespace(ranges=3.5/np.cos(angles),angle_min=-.1,angle_increment=.005,range_min=.1,range_max=12.)
        self.cal=SimpleNamespace(k=[356.7,0.,125.])

    def hit(self):
        return far_face(125.,16.,self.scan,(0.,0.,0.),(0.,0.,0.),self.cal)

    def test_flat_face(self):
        p,n=self.hit()
        np.testing.assert_allclose(p,[3.5,0.],atol=1e-6)
        np.testing.assert_allclose(n,[-1.,0.],atol=1e-6)

    def test_gap(self):
        self.scan.ranges[20]=np.inf
        self.assertIsNone(self.hit())

    def test_nearer_object(self):
        self.scan.ranges[20]=2.
        self.assertIsNone(self.hit())

if __name__=='__main__':unittest.main()
