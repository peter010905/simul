"""Checks for the .57 short-lap candidate."""
from osp_mission.center_wall import CenterWall


def test_lap_budget_is_150_s():
    assert CenterWall().wall_lap.expire(0., 600.) == 150.
