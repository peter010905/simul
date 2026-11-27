""".69: thin SLAM obstacles (legs) are kept; boxes and walls are not."""
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(os.environ.get('OSP_OVERLAY', Path(__file__).resolve().parent / 'overlay'))
sys.path.insert(0, str(ROOT))

from osp_mission.thin_obstacles import thin_obstacle_points  # noqa: E402


def test_legs_kept_boxes_and_walls_dropped():
    data = np.zeros((100, 100), np.int8)
    data[0:2, :] = 100              # wall 5 m
    data[50:52, 50:52] = 100        # leg 0.1 m
    data[20:32, 20:32] = 100        # box 0.6 m
    data[80, 80] = 100              # single-cell speckle
    pts = thin_obstacle_points(data, .05, (1., 2.))
    assert len(pts) == 4
    assert np.allclose(pts[:, 0].mean(), 1. + 51 * .05) and np.allclose(pts[:, 1].mean(), 2. + 51 * .05)
    assert np.allclose(pts[:, 2], .10)


def test_empty_map():
    assert thin_obstacle_points(np.zeros((10, 10), np.int8), .05, (0., 0.)).shape == (0, 3)


if __name__ == '__main__':
    test_legs_kept_boxes_and_walls_dropped()
    test_empty_map()
    print('ok')
