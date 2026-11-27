"""Every LiDAR 5 m Mixed menu shares one camera copy and runs with the three-marker settings."""
import unittest

import robot_board_control as control


class CameraSharingTest(unittest.TestCase):
    def test_lidar5_mixed_menus_share_the_camera(self):
        missing = sorted(action for action, workspace in control.START_WORKSPACES.items()
                         if workspace and '_lidar5_' in workspace and action not in control.FOCUS_CAMERA_ACTIONS)
        self.assertEqual(missing, [])


if __name__ == '__main__':
    unittest.main()
