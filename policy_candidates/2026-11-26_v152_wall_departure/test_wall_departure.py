import numpy as np
from scipy.ndimage import distance_transform_edt
from osp_mission._r32_navigation_core import Grid, connected_nav_cells
from osp_mission.box_access import box_routes
from osp_mission.wall_lap import WallLap


def field():
    cells=np.zeros((100,100),np.int8)
    cells[:,:11]=100
    return Grid(cells,.05,(0.,0.,0.))


def test_30cm_start_reuses_the_existing_safe_departure_corridor():
    g=field();pose=(.825,2.025)
    safe,original=g.reachable(pose,.32)
    assert not safe[g.index(*pose)] and np.isfinite(original[g.index(1.5,2.)])
    nav=connected_nav_cells(g,pose)
    _,parent=box_routes(g,g,nav,safe,pose,(0.,0.,0.))
    joint,fixed=box_routes(g,g,nav,safe,pose,(0.,0.,0.),departure=np.isfinite(original))
    assert not np.isfinite(parent).any()
    assert np.isfinite(fixed[g.index(1.5,2.)])
    assert joint[g.index(*pose)]
    clearance=distance_transform_edt(np.pad(g.cells==0,1))[1:-1,1:-1]*g.resolution
    connector=joint & ~safe
    assert connector.any() and clearance[connector].min() >= .23


def test_observed_local_obstacle_still_blocks_a_departure():
    g=field();pose=(.825,2.025)
    safe,original=g.reachable(pose,.32)
    local=Grid(np.zeros_like(g.cells),g.resolution,g.origin)
    local.cells[g.index(*pose)]=100
    _,fixed=box_routes(g,g,connected_nav_cells(g,pose),safe,pose,(0.,0.,0.),
                      ('map',local),departure=np.isfinite(original))
    assert not np.isfinite(fixed).any()


def test_start_below_existing_minimum_is_held():
    g=field();pose=(.725,2.025)
    safe,original=g.reachable(pose,.32)
    _,fixed=box_routes(g,g,connected_nav_cells(g,pose),safe,pose,(0.,0.,0.),
                      departure=np.isfinite(original))
    assert not np.isfinite(fixed).any()


def test_row_protrusions_preserve_original_collision_geometry():
    cells=np.zeros((100,150),np.int8)
    cells[:3]=100;cells[-3:]=100;cells[:,:3]=100;cells[:,-3:]=100
    for r in range(12,88,12):
        cells[r:r+2,3:6]=100
    g=Grid(cells,.1,(0.,0.,0.))
    before=g.cells.copy()
    safe,dist=g.reachable((1.2,4.),.32)
    lap=WallLap()
    goal=lap.choose(g,dist,(1.2,4.,0.),(0.,0.,0.),9.,600.,lambda _:False)
    assert goal and safe[g.index(goal['x'],goal['y'])]
    np.testing.assert_array_equal(g.cells,before)
