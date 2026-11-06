import xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
import yaml
from osp_mission.frontier import candidates
from osp_mission.world import generate


def test_frontier_is_reachable_and_clear():
    grid=np.full((80,100),-1,dtype=int)
    grid[10:60,10:60]=0
    grid[10:60,60]=100
    # Disconnected free island cannot become a destination.
    grid[10:60,75:95]=0
    result=candidates(grid,.05,(0,0),(1.,1.))
    assert result
    assert all(x<3. for _,x,y,a in result)
    assert not candidates(grid,.05,(0,0),(-1.,-1.))


def test_world_markers_and_no_central_collision(tmp_path):
    scenario=Path(__file__).parents[1]/'config/scenario.yaml'
    path,start=generate(scenario,tmp_path)
    world=ET.parse(path).getroot().find('world')
    assert world.attrib['name']=='osp_studio'
    for model in world.findall('model'):
        if model.attrib['name'].startswith('paint'):
            assert model.find('.//collision') is None
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    for j,ident in enumerate([7,42,103]):
        img=cv2.imread(str(tmp_path/f'marker{j}.png'),0)
        _,ids,_=cv2.aruco.detectMarkers(img,dictionary)
        assert ids.flatten().tolist()==[ident]
    assert start==[1.2,1.3,0.]


def test_random_scenarios_reproducible(tmp_path):
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/scenario.yaml').read_text())
    cfg.update(randomize=True,seed=11)
    p=tmp_path/'scenario.yaml'; p.write_text(yaml.safe_dump(cfg))
    generate(p,tmp_path/'a'); generate(p,tmp_path/'b')
    assert (tmp_path/'a/ground_truth.yaml').read_text()==(tmp_path/'b/ground_truth.yaml').read_text()
