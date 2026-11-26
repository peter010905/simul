"""Checks for the .53 unstick candidate (no ROS node, no robot)."""
import numpy as np

from osp_mission.chair_band import ChairBandTracker


def test_suspected_band_is_avoided_before_commit():
    t = ChairBandTracker()
    t.pending[101] = (np.array([2.0, 0.0]), np.array([-1.0, 0.0]))
    assert t.suspected((1.0, 0.0))
    assert t.suspected((2.2, 0.0))       # within the 0.3 m margin
    assert not t.suspected((4.0, 0.0))
    assert not t.contains((1.0, 0.0))    # not committed


def test_mission_has_no_progress_watch():
    src = open(__import__('osp_mission.mission', fromlist=['x']).__file__).read()
    assert "no_progress_10s" in src and "now-hist[0][0] >= 10." in src
