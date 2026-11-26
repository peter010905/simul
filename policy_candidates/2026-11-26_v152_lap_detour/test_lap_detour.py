"""Checks for the .54 lap-detour candidate (no ROS node, no robot)."""
from osp_mission.center_wall import CenterWall


def test_lap_spacing_and_detour_budget():
    cw = CenterWall()
    assert cw.wall_lap.spacing == 3.0
    assert cw.lap_detours == 0
    assert cw.wall_lap.expire(0., 600.) == 240.
