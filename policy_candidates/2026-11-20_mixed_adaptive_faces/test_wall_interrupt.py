"""Exercise the wall-transit probe and the normal replan entry point without motion."""
import tempfile
import unittest
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time

from osp_mission.phased_hybrid import PhasedHybrid


class WallTransit(unittest.TestCase):
    def test_new_object_interrupts_wall_then_wins_replan(self):
        cells = np.full((80, 100), -1, np.int8)
        cells[5:-5,5:-5] = 100
        cells[6:-6,6:-6] = 0
        cells[30:38,44:51] = 100
        viewed = np.zeros((4, *cells.shape), bool)
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = 13
        msg.info.width, msg.info.height = 100, 80
        msg.info.resolution = .1
        msg.info.origin.orientation.w = 1.
        msg.data = cells.ravel().tolist()
        now = [10.]
        def transform(target, source, stamp):
            result = TransformStamped()
            result.transform.rotation.w = 1.
            if source=='base_link':
                result.transform.translation.x = 3.5
                result.transform.translation.y = 3.6
            return result
        with tempfile.TemporaryDirectory() as tmp:
            params = dict(map_frame='map', base_frame='base_link', duration=600., output_dir=tmp)
            policy = PhasedHybrid()
            policy.drive.costmap = msg
            policy.schedule.phase = 'wall'
            owner = SimpleNamespace(now=lambda:now[0], started=0., map=msg, p=params.get,
                navigation_flow=None, excluded=[], targets={}, tf=SimpleNamespace(lookup_transform=transform),
                known_dock_pose=lambda _:None, get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=now[0])),
                coverage=SimpleNamespace(grid=lambda *args:viewed))
            policy.dispatched(owner, dict(r32_kind='wall_surface', x=1., y=1.))
            self.assertFalse(policy.should_interrupt(owner))
            now[0] = 13.
            self.assertTrue(policy.should_interrupt(owner))
            self.assertEqual(policy.schedule.phase,'wall')
            goal = policy.select(owner,None,cells,viewed,(3.5,3.6,0.),(0.,0.,0.))
            self.assertEqual(goal['r32_kind'],'interior_surface')
            self.assertEqual(policy.schedule.phase,'interior')
            policy.dispatched(owner,goal)
            self.assertTrue(np.isinf(policy.next_box_probe))

if __name__ == '__main__':
    unittest.main()
