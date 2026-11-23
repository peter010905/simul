import numpy as np
import pytest

from osp_mission._r32_navigation_core import Grid
from osp_mission.hybrid import PrintMemory


@pytest.mark.parametrize('resolution', [.05, .08, .1])
def test_print_diversion_budget_is_in_walked_metres(resolution):
    cells = np.zeros((int(4/resolution), int(10/resolution)), dtype=np.int8)
    grid = Grid(cells, resolution, (0., 0., 0.))
    pose = (1., 2., 0.)
    def choose(x, cost):
        memory = PrintMemory()
        for t in (4., 4.2, 4.4):
            memory.observe(dict(point=[x, 2.], observer=pose[:2], stamp=t, size=18.), t)
        return memory.choose(grid, cost, pose, (0.,0.,0.), {}, 4.6, lambda g: False)
    assert choose(6.8, grid) is None
    selected = choose(3.4, grid)
    assert selected is not None
    start = np.array(grid.index(*pose[:2]))
    end = grid.index(selected['x'], selected['y'])
    assert selected['print_route_m'] == pytest.approx(np.abs(start-end).sum()*resolution)
    assert 0 < selected['print_route_m'] <= 2.4
    # The endpoint is in the same global component. Its only costmap path
    # goes around the partition and exceeds the short diversion budget.
    wall = cells.copy()
    wall[int(.5/resolution):, int(1.5/resolution)] = 99
    assert choose(3.4, Grid(wall, resolution, grid.origin)) is None
    # A true near, stationary inspection still needs a free start.
    assert choose(2.2, grid)['stationary']
    blocked = cells.copy()
    blocked[grid.index(*pose[:2])] = 100
    assert choose(2.2, Grid(blocked, resolution, grid.origin)) is None
