"""Exercise the scheduler at an actual decision boundary, with synthetic navigation candidates."""
from unittest.mock import patch

import numpy as np
import pytest

from osp_mission.center_wall import CenterWall
from osp_mission import _r32_navigation_core as r32


def make_scheduler():
    scheduler = CenterWall()
    scheduler.cluster.observed = [(0., 0.)]
    for t in (10., 22., 36.):
        scheduler.dispatched(dict(r32_kind='interior_surface', batch_box=(0., 0.)), t)
    return scheduler


def test_only_recent_camera_supported_repeat_counts():
    s = make_scheduler()
    assert s.cluster.crowded_centers(40.) and not s.cluster.crowded_centers(200.)
    s.cluster.observed = [(1., 0.)]
    assert not s.cluster.crowded_centers(40.)


def test_unseen_fitted_or_neighbour_is_protected():
    s = make_scheduler()
    local = dict(batch_box=(0., 0.), surface_odom=(.4, .1))
    assert s.cluster.local_fragment(local, [(0., 0.)])
    assert not s.cluster.local_fragment(dict(local, fitted_face=1), [(0., 0.)])
    assert not s.cluster.local_fragment(dict(local, batch_box=(.9, 0.)), [(0., 0.)])


@pytest.mark.parametrize('remote_safe,expect_escape', [(True,True), (False,False)])
def test_normal_selector_keeps_safety_and_fallback(remote_safe, expect_escape):
    cells = np.zeros((80,80), dtype=np.int8)
    grid = r32.Grid(cells, .1, (-4.,-4.,0.))
    costmap = r32.Grid(cells.copy(), .1, (-4.,-4.,0.))
    scheduler = make_scheduler()
    choices = [dict(x=x, y=0., distance=abs(x), features=[0,1,0,0,0], stationary=False)
               for x in (.8, 3.)]
    def veto(g):
        return not remote_safe and g['x']>2.
    with patch('osp_mission.center_wall.surface_cells', return_value={}):
        goal = scheduler.decide(grid,costmap,(0.,0.,0.),(0.,0.,0.),
            np.zeros((4,80,80),dtype=bool),40.,600.,veto,choices)
    assert bool(goal.get('cluster_exit')) is expect_escape
    assert goal['x'] == (3. if remote_safe else .8)
