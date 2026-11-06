"""A safe rounded cell is insufficient to justify a continuous shortcut."""

import numpy as np

from osp_mission.fast_sim import motion_blocked
from osp_mission.path_geometry import segment_clear


def test_subcell_shortcut_cannot_slip_inside_the_unchanged_local_stop_radius():
    grid=np.zeros((30,30),dtype=np.int8);grid[:,10]=100
    # Rounded containing cells are three cells from the wall, but the
    # continuous line is only 29.17 cm from its occupied cell centres.
    start=(5.,12.9166666666667);end=(24.,12.9166666666667)
    assert not segment_clear(grid,.1,start,end)
    assert motion_blocked(grid,.1,(start[1]+.5)*.1,(start[0]+.5)*.1)
    assert segment_clear(grid,.1,(5.,14.),(24.,14.))
    for y in np.linspace(5.,24.,100):
        assert not motion_blocked(grid,.1,1.45,(y+.5)*.1)


def test_segment_checks_the_middle_and_does_not_assume_unknown_space_is_free():
    grid=np.zeros((50,50),dtype=np.int8);grid[25,25]=100
    assert not segment_clear(grid,.1,(5.,5.),(45.,45.))
    grid[25,25]=-1
    assert not segment_clear(grid,.1,(5.,5.),(45.,45.))
    grid[25,25]=0
    assert segment_clear(grid,.1,(5.,5.),(45.,45.))
    assert not segment_clear(grid,.1,(-1.,5.),(20.,5.))
