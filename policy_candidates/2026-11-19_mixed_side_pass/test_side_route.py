"""Surface transit can pause; confirmed marker tasks remain protected."""
import unittest
import numpy as np

from osp_mission.side_route import side_face_route


class SideRouteTest(unittest.TestCase):
    def test_mapped_surface_and_box_transit(self):
        self.assertTrue(side_face_route(dict(box_point=[1., 2.])))
        self.assertTrue(side_face_route(dict(face_inspection=True)))
        self.assertTrue(side_face_route(dict(features=np.array([.4, 0., 0., 0., 0.]))))

    def test_generic_goal_and_marker_tasks_remain_protected(self):
        self.assertFalse(side_face_route(dict(features=np.array([.2, 0., 0., 0., 0.]))))
        self.assertFalse(side_face_route(dict(box_point=[1., 2.], approach_id=7)))
        self.assertFalse(side_face_route(dict(face_inspection=True, focus_id=103)))
        self.assertFalse(side_face_route(dict(print_point=[1., 2.], features=np.array([1.]))))
        self.assertFalse(side_face_route(dict(features=np.array([float('nan')]))))


if __name__ == '__main__':
    unittest.main()
