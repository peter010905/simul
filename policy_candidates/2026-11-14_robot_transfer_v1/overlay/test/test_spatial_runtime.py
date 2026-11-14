import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from osp_mission.exploration_memory import ExplorationMemory
from osp_mission.space_geometry import SHAPES, inside_polygon, polygon_area, rasterize_space, triangulate, valid_space
from osp_mission.space_scenarios import make_space_scene
from osp_mission.world import generate


@pytest.mark.parametrize('shape', SHAPES)
def test_accessible_space_and_actual_sdf_match(shape, tmp_path):
    cfg = make_space_scene(1135000000+SHAPES.index(shape), 8, 'mixed', shape)
    assert valid_space(cfg)
    assert cfg['arena']['shape'] == shape
    assert len({t['id'] for t in cfg['targets']}) == 3
    assert all(.08 <= t['size'] <= .30 for t in cfg['targets'])
    vertices = np.array(cfg['arena']['polygon'])
    triangles = triangulate(cfg['arena']['polygon'])
    area = sum(polygon_area(vertices[list(face)].tolist()) for face in triangles)
    assert area == pytest.approx(cfg['arena']['area_m2'])
    scenario = tmp_path/'scene.yaml'
    scenario.write_text(yaml.safe_dump(cfg))
    file, _ = generate(scenario, tmp_path/'world', recording=True)
    world = ET.parse(file).getroot().find('world')
    models = {m.get('name'): m for m in world.findall('model')}
    walls = [m for n, m in models.items() if n.startswith('arena_wall')]
    assert len(walls) == len(cfg['arena']['walls'])
    assert 'north' not in models and 'east' not in models
    assert models['floor'].find('.//visual/geometry/mesh/uri') is not None
    for index, obj in enumerate(cfg['arena']['walls']):
        pose = list(map(float, models[f'arena_wall{index}'].find('pose').text.split()))
        assert pose[:2] == pytest.approx([obj['x'], obj['y']])
        assert pose[-1] == pytest.approx(obj['yaw'])
    occupancy = rasterize_space(cfg)
    if shape in ('circle', 'triangle', 'trapezoid'):
        assert occupancy[-2, 1]  # upper-left bounding-box corner is outside these arenas
    assert not any(name.startswith('paint') for name in models if name != 'floor_markings')


def option(x, y, yaw=0, gain=0):
    return dict(x=x, y=y, yaw=yaw, features=np.array([2., gain, 3., .1, 0.]))


def test_repeat_view_suppressed_but_new_direction_frontier_and_transit_allowed():
    memory = ExplorationMemory()
    first = option(2, 2)
    memory.selected(first, (0, 0, 0))
    memory.completed(True, 10)
    candidates = [first, option(2, 2, math.pi), option(2, 2, 0, .3), option(6, 2)]
    kept = memory.filter(candidates, (0, 0, 0), 30)
    assert len(kept) == 3 and memory.suppressed == 1
    assert len(memory.filter([first], (0, 0, 0), 200)) == 1
    # Map-frame translation/rotation must not erase odometry-anchored history.
    shifted = option(0, 0, -math.pi/2)
    kept = memory.filter([shifted, option(5, 5)], (2, 2, math.pi/2), 30)
    assert len(kept) == 1
    assert memory.filter([first], (0, 0, 0), 30) == []  # no unconditional fallback
    assert len(memory.filter([first], (0, 0, 0), 161)) == 1  # bounded expiry


def test_failed_goal_and_marker_approach_are_never_completed_exploration_views():
    memory = ExplorationMemory()
    memory.selected(option(2, 2), (0, 0, 0))
    memory.completed(False, 10)
    memory.selected(dict(option(2, 2), approach_id=44), (0, 0, 0))
    memory.completed(True, 20)
    assert memory.views == []
    for x, stamp in [(0, 0), (.3, 1), (.7, 2), (1.1, 3), (.7, 20), (.3, 21)]:
        memory.observe(x, 0, stamp)
    assert memory.path_distance_m > 1
    assert memory.revisited_distance_m > 0
