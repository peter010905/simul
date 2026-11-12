import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from osp_mission.space_geometry import SHAPES
from osp_mission.space_scenarios import make_space_scene
from osp_mission.world import generate


@pytest.mark.parametrize('shape',SHAPES)
def test_visual_floor_and_collision_floor_agree_in_world_coordinates(shape,tmp_path):
    config = make_space_scene(1135000000+SHAPES.index(shape),8,'mixed',shape)
    scenario = tmp_path/'scenario.yaml'
    scenario.write_text(yaml.safe_dump(config))
    world,_ = generate(scenario,tmp_path/'world')
    floor = ET.parse(world).getroot().find("world/model[@name='floor']")
    model_pose = np.fromstring(floor.find('pose').text,sep=' ')
    visual = floor.find('link/visual')
    relative_pose = visual.find('pose')
    assert relative_pose is not None, 'World-coordinate mesh needs a compensating relative pose'
    relative = np.fromstring(relative_pose.text,sep=' ')
    mesh_path = visual.find('geometry/mesh/uri').text
    vertices = []
    with open(mesh_path) as stream:
        for line in stream:
            if line.startswith('v '):
                vertices.append(np.fromstring(line[2:],sep=' '))
    vertices = np.array(vertices)
    world_vertices = vertices+model_pose[:3]+relative[:3]
    assert world_vertices[:,:2] == pytest.approx(np.asarray(config['arena']['polygon']))
    assert world_vertices[:,2] == pytest.approx(np.zeros(len(vertices)))
    # The collision slab's top is also at world z=0; cliff protection stays on.
    size = np.fromstring(floor.find('link/collision/geometry/box/size').text,sep=' ')
    assert model_pose[2]+size[2]/2 == pytest.approx(0.)
