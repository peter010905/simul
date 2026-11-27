"""Run against the staged ROS package after sourcing its install/setup.bash."""
import copy
import math
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from osp_mission.escape_control import DriveProgress, ReverseLimit, escape_headings
from osp_mission.local_floor import FLOOR_FRAME, configure_local_floor, floor_transform
from osp_mission.mission import Mission


def scan():
    return SimpleNamespace(ranges=np.full(360, 2.), angle_min=-math.pi,
                           angle_increment=math.pi/180, range_min=.15, range_max=12.)


def test_floor_tracks_only_vertical_offset_at_the_odometry_stamp():
    odom = Odometry()
    odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
    odom.header.stamp.sec, odom.header.stamp.nanosec = 31, 123
    odom.pose.pose.position.x = 7.
    for z in [-.40, .15, -.08]:
        odom.pose.pose.position.z = z
        tr = floor_transform(odom, 'odom', 'base_link')
        assert tr.header == odom.header and tr.child_frame_id == FLOOR_FRAME
        assert (tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z) == (0., 0., z)
        assert tr.transform.rotation.w == 1.
    odom.pose.pose.position.z = math.nan
    assert floor_transform(odom, 'odom', 'base_link') is None
    assert floor_transform(odom, 'map', 'base_link') is None


def test_height_fix_preserves_collision_checks_and_frame_consumers():
    nav = {'local_costmap': {'local_costmap': {'ros__parameters': {
        'global_frame':'odom', 'footprint':'test', 'voxel_layer':{'origin_z':0.}}}},
        'behavior_server':{'ros__parameters':{'local_frame':'odom'}},
        'collision_monitor':{'polygons':['FootprintApproach']}, 'global_costmap':{'frame':'map'}}
    original = copy.deepcopy(nav)
    configure_local_floor(nav)
    assert nav['local_costmap']['local_costmap']['ros__parameters'].pop('global_frame') == FLOOR_FRAME
    original['local_costmap']['local_costmap']['ros__parameters'].pop('global_frame')
    assert nav.pop('behavior_server')['ros__parameters']['local_frame'] == FLOOR_FRAME
    original.pop('behavior_server')
    assert nav == original


def test_watchdog_restarts_on_real_goal_dispatch_and_times_out_a_stationary_goal():
    node = Mock()
    node.drive_progress = DriveProgress()
    node.action_generation = 0
    node.navigation_flow = None
    for t in np.arange(0, 9.5, .5):
        assert not node.drive_progress.stalled(t, (0., 0., 0.))
    Mission.send(node, Mock(), NavigateToPose.Goal(), 'nav')
    assert not node.drive_progress.stalled(10.34, (0., 0., 0.))
    for tick in range(1, 20):
        assert not node.drive_progress.stalled(10.34 + tick*.5, (0., 0., 0.))
    assert node.drive_progress.stalled(20.35, (0., 0., 0.))
    assert not node.drive_progress.stalled(24., (0., 0., 0.))


def test_productive_turn_across_pi_and_missing_odometry():
    w = DriveProgress()
    for i in range(28):
        assert not w.stalled(i*.5, (0., 0., math.remainder(3.+i*.07, math.tau)))
    assert not w.stalled(14., None)
    assert len(w.samples) == 0


def test_escape_requires_supported_path_and_checks_flanks_outside_the_sector():
    m = scan()
    assert escape_headings(m, (0.,0.,0.), .19)[0] == 0.
    m.ranges[:] = np.inf
    assert not escape_headings(m, (0.,0.,0.), .19)
    m.ranges[:] = 2.
    m.ranges[125] = .21  # Forward-right; outside the old 15-degree sector.
    assert not escape_headings(m, (0.,0.,0.), .19)
    m.ranges[:] = 2.
    m.ranges[15] = .22   # Behind the base: a straight departure increases clearance.
    assert escape_headings(m, (0.,0.,0.), .19) == [0.]


def test_reverse_limit_requires_release_fresh_data_and_forward_distance():
    gate = ReverseLimit()
    gate.report(True, 0.); gate.odometry((3.,4.,math.pi/2),0.)
    gate.report(True, .5); gate.odometry((3.,3.9,math.pi/2),.5)
    gate.report(False, 1.)
    for t, pose in [(1.1,(3.,4.1,0.)),(1.8,(3.3,3.9,0.)),(4.,(3.,4.2,0.))]:
        gate.odometry(pose, t)
        assert gate.latched
    gate.report(False,4.1); gate.odometry((3.,4.02,math.pi/2),4.1)
    assert not gate.latched


def test_forward_progress_before_hardware_releases_the_limit_counts():
    gate = ReverseLimit()
    gate.report(True, 0.); gate.odometry((0.,0.,0.),0.)
    gate.report(True, .5); gate.odometry((-.08,0.,0.),.5)
    gate.report(True, 1.); gate.odometry((.01,0.,0.),1.)
    assert gate.latched
    gate.report(False, 1.1); gate.report(False, 1.7)
    gate.odometry((.01,0.,0.),1.7)
    assert not gate.latched


def test_recovery_keeps_rear_scan_action_server_and_cooldown_interlocks():
    node = Mock()
    node.reverse_limit = ReverseLimit()
    node.rear_clear = True
    node.last_recovery = 0.
    node.sensor_fresh.return_value = node.backup.server_is_ready.return_value = True
    node.navigation_flow.backup_allowed.return_value = True
    assert Mission.can_retreat(node, 20.)
    assert not Mission.can_retreat(node, 14.)
    node.reverse_limit.report(True, 20.)
    assert not Mission.can_retreat(node, 20.)
    node.reverse_limit = ReverseLimit()
    node.navigation_flow.backup_allowed.return_value = False
    assert not Mission.can_retreat(node, 20.)
    node.navigation_flow.backup_allowed.return_value = True
    node.sensor_fresh.return_value = False
    assert not Mission.can_retreat(node, 20.)
    node.sensor_fresh.return_value = True
    node.rear_clear = False
    assert not Mission.can_retreat(node, 20.)
    node.rear_clear = True
    node.backup.server_is_ready.return_value = False
    assert not Mission.can_retreat(node, 20.)
