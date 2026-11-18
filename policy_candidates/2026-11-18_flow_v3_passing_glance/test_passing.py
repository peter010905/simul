import unittest

import cv2
import numpy as np

from passing import PassingGlance, printed_at


class PassingTests(unittest.TestCase):
    def setUp(self):
        self.image = np.full((250,250),235,np.uint8)
        self.policy = PassingGlance()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

    def update(self, t, contours=(), confirmed=None):
        return self.policy.update(t,t,(0.,0.,0.),self.image,356.7,123.7,contours,confirmed)

    def test_single_full_tag_and_rerequest(self):
        for ident in (7,103,349,812):
            draw = getattr(cv2.aruco,'generateImageMarker',None) or cv2.aruco.drawMarker
            tag = draw(self.dictionary,ident,35)
            for rot in range(4):
                self.image[93:128,12:47] = np.rot90(tag,rot)
                self.assertTrue(printed_at(self.image,(29.,110.,34.)))
        self.assertEqual(self.update(1,[(29.,110.,34.)])['yaw_delta_deg'],6.)
        self.assertIsNone(self.update(1.1,[(29.,110.,34.)]))
        self.assertIsNone(self.update(17,[(29.,110.,34.)]))

    def test_plain_ring_and_crop(self):
        cv2.rectangle(self.image,(12,93),(46,127),18,5)
        self.assertIsNone(self.update(1,[(29.,110.,34.)]))
        self.image[:,0:20]=12
        self.assertFalse(printed_at(self.image,(6.,110.,34.)))

    def test_confirmed_view_offset(self):
        cue=dict(action='hold',uv=[143.,140.],yaw_delta_deg=-3.,dwell_s=.5)
        request=self.update(1,confirmed=cue)
        self.assertEqual(request['action'],'turn')
        self.assertAlmostEqual(request['yaw_delta_deg'],-3.1,places=1)
        self.assertEqual(request['dwell_s'],.6)
        self.assertEqual(cue['action'],'hold')
        self.assertEqual(self.update(14,confirmed=dict(cue,uv=[124.,140.]))['action'],'hold')


if __name__=='__main__':
    unittest.main()
