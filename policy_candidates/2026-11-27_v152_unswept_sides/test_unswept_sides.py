""".63: a box's unswept face directions come before re-sweeping a swept one; generic interior
samples on a wall-sized structure are dropped, box-sized islands are kept."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / 'overlay'))

from osp_mission.box_continuity import BoxContinuity  # noqa: E402
from osp_mission.center_wall import on_wall, wall_sized_distance  # noqa: E402
from osp_mission import _r32_navigation_core as r32  # noqa: E402


def face(x, y, normal=0.0, box=(0.0, 0.0), view=None, attempts=0):
    view = view or (x + math.cos(normal), y + math.sin(normal))
    return dict(r32_kind='interior_surface', face_inspection=True, surface_odom=[x, y],
                normal_odom=normal, batch_box=tuple(box), previous_attempts=attempts,
                x=view[0], y=view[1])


def sweep(c, goal, t):
    c.dispatched(goal, t)
    c.begin_sweep(1.0, t + 1)
    c.result('spin', 4, '', t + 3)


def test_unswept_sides_first():
    c = BoxContinuity()
    east, north = face(.4, 0.), face(0., .4, math.pi / 2)
    west_other_box = face(2.6, 0., math.pi, box=(3., 0.))
    sweep(c, east, 10.)
    # same side again from a new standoff (allowed by .62) waits while north is unswept
    again = dict(east, x=1.4, y=.6)
    ranked = [(1., again), (2., north), (3., west_other_box)]
    assert [g for _, g in c.unswept_first(ranked)] == [north, west_other_box]
    # once every offered side is swept, the swept side is available again
    sweep(c, north, 20.)
    assert [g for _, g in c.unswept_first([(1., again)])] == [again]
    # another box is never held back by this box's sweeps
    assert c.unswept_first([(1., west_other_box)]) == [(1., west_other_box)]


def test_unswept_side_may_follow_one_failure():
    import osp_mission.box_continuity as bc
    bc.short_tour = lambda options, grid, safe, remaining: min(options, key=lambda r: r[0])[1]
    c = BoxContinuity()
    east = face(.4, 0.)
    sweep(c, east, 10.)
    c.box, c.entered = (0., 0.), 10.
    failed_once = face(-.4, 0., math.pi, attempts=1)
    failed_twice = face(0., -.4, -math.pi / 2, attempts=2)
    swept_failed = dict(east, x=1.4, y=.6, previous_attempts=1)
    chosen = c.choose([(1., failed_once)], 20., 500., None, None)
    assert chosen is not None and chosen['surface_odom'] == failed_once['surface_odom']
    assert c.choose([(1., failed_twice)], 20., 500., None, None) is None
    assert c.choose([(1., swept_failed)], 20., 500., None, None) is None


def test_wall_sized_structure():
    res = .05
    cells = np.zeros((80, 120), np.int8)
    cells[5, :] = 100                  # 6 m wall
    cells[40:52, 50:62] = 100           # 0.6 m box
    grid = r32.Grid(cells, res, (0., 0., 0.))
    dist = wall_sized_distance(grid)
    assert on_wall(grid, dist, grid.world(5, 30))
    assert on_wall(grid, dist, grid.world(8, 30))           # 0.15 m off the wall
    assert not on_wall(grid, dist, grid.world(12, 30))      # 0.35 m off
    assert not on_wall(grid, dist, grid.world(40, 55))      # box face
    empty = r32.Grid(np.zeros((20, 20), np.int8), res, (0., 0., 0.))
    assert not on_wall(empty, wall_sized_distance(empty), (0.5, 0.5))


if __name__ == '__main__':
    test_unswept_sides_first()
    test_unswept_side_may_follow_one_failure()
    test_wall_sized_structure()
    print('ok')
