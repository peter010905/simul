"""Camera evidence must not silently complete unseen obstacle faces."""

import math

import numpy as np

from osp_mission.surface_coverage import DirectionalCoverage


def test_camera_needs_three_distinct_frames_and_leaves_opposite_face_unseen():
    grid = np.zeros((40, 60), dtype=np.int8)
    grid[3:37, 30] = 100
    memory = DirectionalCoverage()
    endpoints = [[3.05, 2.05]] * 100
    args = (grid, .1, (0., 0.), (1.05, 2.05, 0.), endpoints)
    for stamp in [0., .01, .02, .1, .2, .21]:
        memory.observe(*args, stamp, 665., 1.25)
    assert not memory.viewed.any()
    memory.observe(*args, .4, 665., 1.25)
    assert memory.viewed[[7, 0, 1], 20, 30].all()
    assert not memory.viewed[4, 20, 30]
    assert memory.viewed.sum() == 3
    for stamp in [.6, .8, 1.]:
        memory.observe(grid, .1, (0., 0.), (5.05, 2.05, math.pi),
                       endpoints, stamp, 665., 1.25)
    assert memory.viewed[4, 20, 30]


def test_occluded_or_out_of_view_surfaces_are_not_completed():
    grid = np.zeros((40, 60), dtype=np.int8)
    grid[3:37, 30] = 100
    grid[3:37, 20] = 100
    memory = DirectionalCoverage()
    for stamp in [0., .2, .4]:
        memory.observe(grid, .1, (0., 0.), (1.05, 2.05, 0.),
                       [[3.05, 2.05]], stamp, 665., 1.25)
    assert not memory.viewed.any()
    for stamp in [.6, .8, 1.]:
        memory.observe(grid, .1, (0., 0.), (1.05, 2.05, math.pi),
                       [[2.05, 2.05]], stamp, 665., 1.25)
    assert not memory.viewed.any()


def test_slam_grid_shift_clears_coverage_instead_of_marking_other_obstacles():
    grid = np.zeros((40, 60), dtype=np.int8)
    grid[3:37, 30] = 100
    memory = DirectionalCoverage()
    for stamp in [0., .2, .4]:
        memory.observe(grid, .1, (0., 0.), (1.05, 2.05, 0.),
                       [[3.05, 2.05]], stamp, 665., 1.25)
    assert memory.viewed.any()
    assert not memory.grid(grid, .1, (-1., 0.)).any()


def test_endpoint_mode_requires_independent_front_facing_camera_frames():
    from osp_mission.surface_coverage import SurfaceCoverage
    grid=np.zeros((40,60),dtype=np.int8);grid[5:35,30:35]=100
    memory=SurfaceCoverage(directional=False)
    for stamp in [0.,.01,.2]:
        memory.observe(grid,.1,(0.,0.),(1.05,2.05,0.),[[3.05,2.05]],stamp,665.,1.25)
    assert memory.viewed.shape==grid.shape and not memory.viewed.any()
    memory.observe(grid,.1,(0.,0.),(1.05,2.05,0.),[[3.05,2.05]],.4,665.,1.25)
    assert memory.viewed[20,30] and not memory.viewed[20,34]
