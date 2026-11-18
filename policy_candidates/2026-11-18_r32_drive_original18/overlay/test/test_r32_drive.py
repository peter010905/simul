from pathlib import Path
from copy import deepcopy

from ament_index_python.packages import get_package_share_directory
import yaml
import pytest

from osp_mission.r32_drive import R32Drive, in_map
from osp_mission.r32_nav_config import configure


def test_drive_config_changes_controller_without_replacing_collision_or_sensor_settings():
    file = Path(get_package_share_directory('turtlebot4_navigation'))/'config/nav2.yaml'
    vendor = yaml.safe_load(file.read_text())
    effective = deepcopy(vendor)
    safety = deepcopy(effective['collision_monitor'])
    configure(effective, vendor)
    follow = effective['controller_server']['ros__parameters']['FollowPath']
    assert 'MPPI' in follow['plugin'] and follow['vx_min'] == 0
    assert follow['vx_max'] == .25 and follow['wz_max'] == .5
    assert follow['PathAlignCritic']['cost_weight'] == 3
    assert effective['collision_monitor'] == safety
    assert not effective['planner_server']['ros__parameters']['GridBased']['allow_unknown']


def test_missing_costmap_does_not_fabricate_a_route():
    drive = R32Drive()
    assert drive.choose(None, None, None, None, None) is None
    assert drive.diagnostic['reason'] == 'waiting_for_global_costmap'


def test_failed_goal_anchor_converts_back_to_current_map_frame():
    from math import pi
    assert in_map(('odom',3.,5.), (3.,4.,pi/2)) == pytest.approx((1.,0.))
    assert in_map(('odom',3.,5.), None) is None
