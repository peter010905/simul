import math
import numpy as np
import pytest

from osp_mission.surface_sweep import endpoint, extend_turn


@pytest.mark.parametrize('side', [-1, 1])
def test_follow_visible_wall_extent_on_either_side(side):
    points = np.column_stack((np.linspace(-.75, .75, 65), np.full(65, side*.8)))
    center = (0, side*.8)
    cam = (0, 0, 0)
    tip = endpoint(points, center, cam)
    turn = side*math.pi/2
    result = extend_turn(turn, tip, cam, math.radians(60))
    assert result == pytest.approx(turn + side*math.radians(20))
    assert extend_turn(turn, None, cam, 1.) == turn


def test_ignore_tiny_fragments_and_clip_rotation():
    cam = (0, 0, 0)
    points = np.column_stack((np.linspace(-.05, .05, 12), np.full(12, .8)))
    assert endpoint(points, (0, .8), cam) is None
    assert extend_turn(math.radians(110), (-1, 0), cam, 2.) == pytest.approx(math.radians(130))
    assert extend_turn(1., (1, 0), cam, 1.) == 1.
