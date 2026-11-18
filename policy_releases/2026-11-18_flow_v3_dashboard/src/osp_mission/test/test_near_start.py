import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from osp_mission.edge_policy import options
from osp_mission.near_start import open_start_region, validate_radius


def test_exception_cannot_change_graph_outside_thirty_centimetres():
    safe = np.zeros((41, 41), dtype=bool)
    original = safe.copy()
    result = open_start_region(safe, .05, (0., 0.), (1.025, 1.025), .30)
    y, x = np.indices(safe.shape)
    radius = np.hypot((x+.5)*.05-1.025, (y+.5)*.05-1.025)
    assert result[20, 20]
    assert result.any() and not result[radius > .30+1e-12].any()
    assert np.array_equal(safe, original)


@pytest.mark.parametrize('radius', [-.01, .301, float('nan'), float('inf')])
def test_exception_rejects_invalid_or_larger_radius(radius):
    with pytest.raises(ValueError):
        validate_radius(radius)


def test_reconnects_start_without_erasing_map_or_relaxing_destinations():
    data = np.zeros((80, 80), dtype=np.int16)
    data[[0, -1], :] = 100
    data[:, [0, -1]] = 100
    # A nearby return and an unknown start hole disconnect the original graph.
    data[40, 38] = 100
    data[39, 40] = -1
    original = data.copy()
    robot = (2.025, 2.025)
    baseline, _ = options(data, .05, (0., 0.), robot)
    choices, _ = options(data, .05, (0., 0.), robot, start_ignore_radius=.30)
    assert not baseline and choices
    clearance = distance_transform_edt(original <= 50)*.05
    for c in choices:
        assert original[c['cell']] == 0
        assert clearance[c['cell']] >= .32
        assert np.hypot(c['x']-robot[0], c['y']-robot[1]) > .30
    assert np.array_equal(data, original)


def test_exception_cannot_cross_a_wall_outside_the_start_region():
    data = np.zeros((80, 80), dtype=np.int16)
    data[[0, -1], :] = 100
    data[:, [0, -1]] = 100
    data[:, 46] = 100
    data[39, 40] = -1
    choices, _ = options(data, .05, (0., 0.), (2.025, 2.025), start_ignore_radius=.30)
    assert choices
    assert all(c['cell'][1] < 46 for c in choices)


def test_zero_radius_preserves_a_disconnected_start():
    data = np.zeros((60, 60), dtype=np.int16)
    data[30, 30] = 100
    choices, _ = options(data, .05, (0., 0.), (1.525, 1.525), start_ignore_radius=0.)
    assert not choices
