"""Checks for the .52 robust-lap candidate (no ROS node, no robot)."""
import numpy as np

from osp_mission.chair_band import ChairBandTracker
from osp_mission.work_rules import FaceWork, RULES
from osp_mission.wall_lap import WallLap


def tracker_with_wall():
    t = ChairBandTracker()
    t.walls[1] = (np.array([2.0, 0.0]), np.array([-1.0, 0.0]))  # chairs at x < 2
    return t


def test_projection_moves_chair_face_stops_in_front_of_the_wall():
    t = tracker_with_wall()
    assert np.allclose(t.project((1.5, 3.0), 1.0), (3.0, 3.0))   # beyond the wall
    assert np.allclose(t.project((2.4, -1.0), 1.0), (3.0, -1.0))  # too close in front
    assert t.project((4.0, 0.0), 1.0) is None                    # already clear


def test_projected_stops_are_not_vetoed():
    t = tracker_with_wall()
    q = t.project((1.0, 0.0), 1.0)
    assert not t.contains(q, .3)


def test_face_reconsideration_is_bounded():
    work = FaceWork()
    geometry = [0] * 40
    for k in range(10):
        geometry = [(k + i) % 2 for i in range(40)]  # every SLAM update changes the face
        row = work.probe((1.0, 1.0), 0.0, geometry)
        row['failures'].append(float(k))
    assert row['reconsiderations'] == RULES.face_reconsiderations
    assert len(row['failures']) > RULES.face_retries_per_geometry
    assert not work.allowed((1.0, 1.0), 0.0, 1000.)


def test_lap_has_projection_hook():
    assert WallLap().project is None
