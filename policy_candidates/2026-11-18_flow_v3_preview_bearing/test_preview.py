import unittest
from types import SimpleNamespace
from preview_geometry import preview_geometry


class PreviewTest(unittest.TestCase):
    def setUp(self):
        self.info=SimpleNamespace(width=250,height=250,k=[199.2553,0.,123.6739,0.,199.2553,127.,0.,0.,1.],
                                  header=SimpleNamespace(frame_id='oakd_rgb_camera_optical_frame'))

    def test_model_is_local_to_the_preview(self):
        corrected=preview_geometry(self.info)
        self.assertAlmostEqual(corrected.k[0],356.666987)
        self.assertAlmostEqual(self.info.k[0],199.2553)
        self.assertEqual(corrected.k[1:],self.info.k[1:])
        self.info.header.frame_id='other_camera'
        self.assertIs(preview_geometry(self.info),self.info)

    def test_another_resolution_or_calibration_is_not_rescaled(self):
        self.info.width=640
        self.assertIs(preview_geometry(self.info),self.info)
        self.info.width=250
        self.info.k[0]=356.
        self.assertIs(preview_geometry(self.info),self.info)
