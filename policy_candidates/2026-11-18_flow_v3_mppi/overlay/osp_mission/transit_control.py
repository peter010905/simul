"""Local path tracking for the Flow v3 mission."""
from collections import deque
from copy import deepcopy
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from osp_mission.navigation_flow import NavigationFlow


def configure_controller(nav, vendor):
    settings = nav['controller_server']['ros__parameters']
    controller = deepcopy(vendor['controller_server']['ros__parameters']['FollowPath'])
    if controller['plugin'] != 'nav2_mppi_controller::MPPIController':
        raise RuntimeError('Expected the Jazzy TurtleBot4 MPPI controller')
    controller.update(vx_max=.25, vx_min=0., wz_max=.5, visualize=False)
    controller['PathAlignCritic']['cost_weight'] = 3.
    controller['GoalAngleCritic']['threshold_to_consider'] = .25
    settings['FollowPath'] = controller
    # Allow an actual alignment to complete; the mission bounds time without translation.
    settings['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
                                        required_movement_angle=.35)
    nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
        Path(get_package_share_directory('nav2_bt_navigator')) /
        'behavior_trees/navigate_w_replanning_time.xml')


class TransitFlow(NavigationFlow):
    def __init__(self, *args):
        super().__init__(*args)
        self.headings = deque()

    def begin_navigation(self):
        super().begin_navigation()
        self.headings.clear()

    def observe_heading(self, rotation, now):
        yaw = math.atan2(2*(rotation.w*rotation.z + rotation.x*rotation.y),
                         1-2*(rotation.y**2+rotation.z**2))
        if not math.isfinite(yaw):
            return
        while self.headings and now - self.headings[0][0] > 2.5:
            self.headings.popleft()
        self.headings.append((now, yaw))

    def stalled(self, now):
        if not super().stalled(now):
            return False
        turning = (len(self.headings) > 1 and now-self.headings[-1][0] < 1.
                   and abs(math.remainder(self.headings[-1][1]-self.headings[0][1], 2*math.pi)) >= .15)
        return not turning or now-self.progress_time >= 16.

    def report(self):
        return dict(super().report(), controller='MPPI', turning_stall_timeout_s=16.)
