import unittest
from glance import EdgeGlance


class GlanceTest(unittest.TestCase):
    def setUp(self):
        self.policy=EdgeGlance()

    def frame(self,t,x=None,confirmed=None,pose=(0.,0.,0.)):
        contours=[] if x is None else [(x,120.,15.)]
        return self.policy.update(t,t,pose,250,356.7,123.7,contours,confirmed)

    def test_edge_angle_and_duration(self):
        for x,sign in [(20,1),(230,-1)]:
            self.policy=EdgeGlance()
            response=self.frame(1,x)
            self.assertEqual(response['yaw_delta_deg'],sign*6)
            self.assertEqual(response['dwell_s'],.3)
            self.assertNotIn('xy',response)

    def test_center_and_repeat(self):
        self.assertIsNone(self.frame(1,125))
        self.assertIsNotNone(self.frame(2,22))
        self.assertIsNone(self.frame(2,230))
        self.assertIsNone(self.frame(3,230))
        self.assertIsNone(self.frame(16,20))
        self.assertIsNotNone(self.frame(17,230))

    def test_confirmed_priority_and_merge(self):
        confirmed={'action':'turn','source':'confirmed'}
        self.assertIs(self.frame(1,15,confirmed),confirmed)
        self.assertIsNotNone(self.frame(14,15))
        self.assertIsNone(self.frame(14.1,18,confirmed))
        self.assertIs(self.frame(15,18,confirmed),confirmed)

if __name__=='__main__':unittest.main()
