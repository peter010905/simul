import numpy as np
from scipy import ndimage

from osp_mission._r32_navigation_core import Grid
from osp_mission.adaptive_faces import partial_face_pose


def test_short_unknown_strip_and_navigation_guards():
    grid = Grid(np.zeros((100, 100), dtype=np.int8), .05, (-2.5, -2.5, 0.))
    face = np.array([.025, .025])
    normal = np.array([1., 0.])
    for y in np.arange(-.36, .38, .025):
        grid.cells[grid.index(*[.025, y])] = 100
    for x in np.arange(.075, .30, .025):
        for y in np.arange(-.09, .10, .025):
            grid.cells[grid.index(x, y)] = -1
    pose = (1.2, .5, 0.)
    fields = dict(scan_angle=.42, surface_odom=list(face))
    clearance = ndimage.distance_transform_edt(grid.cells == 0) * grid.resolution
    nav, distances = grid.reachable(pose[:2], .32)

    def propose(mask=nav, veto=lambda _: False, walk=2.7):
        return partial_face_pose(grid, grid, mask, distances, clearance, face, normal,
                                 .32, pose, fields, veto, walk)

    view = propose()
    assert view and view[0] <= 2.7
    assert view[1]['face_frontier_gap_m'] > .15
    assert view[1]['surface_odom'] == list(face)
    assert view[1]['surface_map'] == list(face)
    assert 'face_frontier_gap_m' not in fields
    assert np.linalg.norm(np.array([view[1]['x'], view[1]['y']]) - face) <= 1.5
    assert propose(np.zeros_like(nav)) is None
    assert propose(veto=lambda _: True) is None
    distances += 10.
    assert propose() is None
    distances -= 10.
    grid.cells[grid.index(*(face + .12 * normal))] = 100
    assert propose() is None
    grid.cells[grid.index(*(face + .12 * normal))] = -1
    grid.cells[grid.index(*(face + .225 * normal))] = 100
    assert propose() is None
    grid.cells[:, grid.index(.05, 0.)[1]:grid.index(.75, 0.)[1]] = -1
    assert propose() is None
