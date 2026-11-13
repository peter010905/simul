"""Concave world geometry and hidden-target accessibility are real constraints."""
import math
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt
from scipy.sparse.csgraph import dijkstra
from osp_mission.policy import graph
from osp_mission.challenge_scenarios import SHAPES,make_challenge_arena,make_challenge_scene,rectangle_union
from osp_mission.space_geometry import polygon_area,triangulate,rasterize_space,valid_space


@pytest.mark.parametrize('shape',SHAPES)
def test_actual_outer_boundary_is_concave_and_forces_long_detour(shape):
    arena,start,far=make_challenge_arena(np.random.default_rng(40),shape)
    cfg=dict(arena=arena,start=start,obstacles=[],targets=[])
    assert len(arena['polygon'])>=8 and polygon_area(arena['polygon'])>45
    assert len(triangulate(arena['polygon']))==len(arena['polygon'])-2
    assert len(arena['walls'])==len(arena['polygon'])
    occupied=rasterize_space(cfg,.15)
    safe=distance_transform_edt(~occupied)*.15>=.34
    distances=dijkstra(graph(safe,.15),directed=False,
        indices=int(start[1]/.15)*safe.shape[1]+int(start[0]/.15))
    detour=distances[int(far[1]/.15)*safe.shape[1]+int(far[0]/.15)]
    assert math.isfinite(detour)
    assert detour>1.7*math.hypot(far[0]-start[0],far[1]-start[1])
    # The direct shortcut must cross the exterior, not just a painted line.
    ys=np.floor(np.linspace(start[1],far[1],300)/.15).astype(int)
    xs=np.floor(np.linspace(start[0],far[0],300)/.15).astype(int)
    assert occupied[ys,xs].any()


@pytest.mark.parametrize('shape',SHAPES)
def test_exact_obstacle_count_and_three_hidden_approachable_targets(shape):
    cfg=make_challenge_scene(1192300101,12,shape=shape)
    assert cfg==make_challenge_scene(1192300101,12,shape=shape)
    assert len(cfg['obstacles'])==cfg['difficulty']==12
    assert len({t['id'] for t in cfg['targets']})==3
    assert len({t['carrier_obstacle'] for t in cfg['targets']})==3
    assert valid_space(cfg,.1) and valid_space(cfg,.15)
    for t in cfg['targets']:
        assert t['mount']=='obstacle' and t['hidden_from_start']
        assert (cfg['start'][0]-t['x'])*math.cos(t['yaw'])+(cfg['start'][1]-t['y'])*math.sin(t['yaw'])<-.2
    assert not cfg['challenge']['policy_receives_ground_truth']


def test_union_rejects_disconnected_geometry_and_invalid_requests():
    with pytest.raises(ValueError):rectangle_union([(0,0,1,1),(3,3,4,4)])
    with pytest.raises(ValueError):make_challenge_scene(1,shape='not_a_shape')
    with pytest.raises(ValueError):make_challenge_scene(1,profile='walls')


def test_coarse_approach_grid_regression():
    cfg=make_challenge_scene(1192501010,16,shape='u_shape')
    assert valid_space(cfg,.1) and valid_space(cfg,.15)
