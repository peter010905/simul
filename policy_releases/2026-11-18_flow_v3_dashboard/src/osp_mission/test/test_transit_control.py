from copy import deepcopy
from math import sin, cos
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from osp_mission.transit_control import configure_controller, TransitFlow


def test_motion_only_changes_and_no_automatic_spin_retry():
    p = Path(get_package_share_directory('turtlebot4_navigation'))/'config/nav2.yaml'
    vendor = yaml.safe_load(p.read_text())
    nav = deepcopy(vendor)
    configure_controller(nav, vendor)
    for key in ('planner_server','collision_monitor','local_costmap','global_costmap','velocity_smoother'):
        assert nav[key] == vendor[key]
    controller = nav['controller_server']['ros__parameters']['FollowPath']
    assert 'MPPI' in controller['plugin'] and controller['PathAlignCritic']['cost_weight'] == 3.
    assert controller['vx_max'] == .25 and controller['wz_max'] == .5
    tree = ET.parse(nav['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
    assert tree.find('.//FollowPath') is not None and tree.find('.//Spin') is None


def test_odom_turn_buys_finite_time_but_stationary_or_stale_does_not():
    flow=TransitFlow()
    for speed in (.25,0.):
        flow.begin_navigation()
        for t in range(17):
            flow.observe_position(0.,0.,t)
            flow.observe_heading(NS(x=0.,y=0.,z=sin(t*speed/2),w=cos(t*speed/2)),t)
            if t==9: assert flow.stalled(t) == (speed==0.)
        assert flow.stalled(16)
    flow.begin_navigation()
    flow.observe_position(0.,0.,20.)
    assert flow.stalled(28.)
