import math
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from osp_mission._r32_navigation_core import Grid
from osp_mission.regional_surface import RegionalSurface


def run(wall=True, box=True, frontier=None, reserved=None, block_box=False):
    grid = Grid(np.zeros((60, 60), dtype=np.int8), .1, (-3., -3., 0.))
    safe, walk = grid.reachable((0., 0.), .32)
    state = RegionalSurface()
    survey = SimpleNamespace(camera=(443., 1.2), reference_m=.14, readable_px=18.,
                             wall_key=lambda g: (0, 0, 0), last_direction={})
    def view(x, kind):
        return dict(x=x, y=0., yaw=0., scan_angle=.4, surface_map=[x+.5,0.],
                    normal_odom=math.pi, surface_odom=[x+.5,0.], r32_kind=kind+'_surface')
    walls = [(0.,view(.1,'wall'))] if wall else []
    objects = [(2.,dict(view(1.8,'interior'), batch_box=(2.3, 0.)))] if box else []
    if block_box:
        walk[grid.index(1.8,0.)] = math.inf
    if reserved:
        reserved = dict(walls[0][1],survey_wall=[0,0,0])
    else:
        reserved = None
    if frontier is not None:
        frontier=dict(x=frontier,y=0.,features=[1.,1.],r32_kind='interior_frontier')
    with patch('osp_mission.regional_surface.projected_coverage', side_effect=lambda grid,v,*a:{v.key:v.weight}):
        return state.choose(grid,safe,walk,(0.,0.,0.),survey,walls,objects,
                            reserved,30.,600.,frontier=frontier,heading=0.)


def test_fresh_box_is_served_with_walls_present():
    assert run()['r32_kind'] == 'interior_surface'


def test_wall_only_and_unreachable_box_still_receive_wall_service():
    assert run(box=False)['r32_kind'] == 'wall_surface'
    assert run(block_box=True)['r32_kind'] == 'wall_surface'


def test_wall_reservation_can_win_over_box():
    assert run(reserved=True)['r32_kind'] == 'wall_surface'


def test_open_space_after_wall_and_reject_unreachable_frontier():
    assert run(box=False,frontier=1.2)['r32_kind'] == 'interior_frontier'
    assert run(box=False,frontier=100.)['r32_kind'] == 'wall_surface'


def test_dispatch_never_refreshes_camera_direction_without_exposure():
    from osp_mission.visual_survey import VisualSurvey
    from osp_mission.edge_policy import DIRECTIONS
    survey = VisualSurvey()
    key = (0, 0, 0)
    survey.wall_times[key] = (0., -float('inf'))
    survey.dispatched(dict(survey_wall=key), 15.)
    assert not survey.last_direction
    grid = Grid(np.zeros((40,40),dtype=np.int8), .1, (-2.,-2.,0.))
    viewed = np.zeros((4,40,40),dtype=bool)
    samples = {('wall',1,1,0,0):[(20,20),(21,20)]}
    survey.progress(samples, grid, viewed, (0.,0.,0.), (0.,0.,0.), 16., DIRECTIONS)
    assert not survey.last_direction
    viewed[1,20,20] = True
    survey.progress(samples, grid, viewed, (0.,0.,0.), (0.,0.,0.), 17., DIRECTIONS)
    assert survey.last_direction == {0:17.}
    survey.progress(samples, grid, viewed, (0.,0.,0.), (0.,0.,0.), 18., DIRECTIONS)
    assert survey.last_direction == {0:17.}
