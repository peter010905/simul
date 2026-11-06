import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from osp_mission.scenarios import make_scenario,valid,rasterize
from osp_mission.policy import BASELINE,FEATURES,load_policy,options,choose,scan_angle,graph
from osp_mission.fast_sim import SearchEnv
from osp_mission.world import generate
import yaml


def test_scenario_reproducibility_access_and_world_geometry(tmp_path):
    for seed in [0,1,2,3]:
        a=make_scenario(seed); b=make_scenario(seed)
        assert a==b and valid(a)
        assert len(a['targets'])==3
        assert len({t['id'] for t in a['targets']})==3
        assert len(a['obstacles'])>=8
    source=tmp_path/'scene.yaml'; source.write_text(yaml.safe_dump(a))
    world,_=generate(source,tmp_path/'world')
    obstacles=[m for m in ET.parse(world).findall('world/model') if m.attrib['name'].startswith('obstacle')]
    assert len(obstacles)==len(a['obstacles'])
    assert all(o.find('.//collision') is not None for o in obstacles)


def test_policy_observes_only_map_and_returns_clear_reachable_goal():
    data=np.full((45,65),-1,dtype=np.int8); data[4:40,4:55]=0; data[4:40,28]=100
    c,_=options(data,.1,(0,0),(1.,1.))
    assert c and all(item['x']<2.8 for item in c)
    goal=choose(c,BASELINE); assert data[goal['cell']]==0
    assert math.isfinite(goal['yaw'])
    assert 2*math.pi/3<=scan_angle(BASELINE)<=2*math.pi


def test_policy_schema_and_finite_parameters(tmp_path):
    path=tmp_path/'policy.json'
    path.write_text(json.dumps(dict(features=FEATURES,parameters=BASELINE.tolist())))
    np.testing.assert_array_equal(load_policy(path),BASELINE)
    path.write_text(json.dumps(dict(features=FEATURES,parameters=[float('nan')]*len(BASELINE))))
    with pytest.raises(ValueError): load_policy(path)


def test_sensors_do_not_reveal_hidden_target_or_cells_behind_wall():
    cfg=dict(seed=7,start=[1.2,1.3,0.],obstacles=[dict(type='box',x=3.,y=5.5,sx=.3,sy=11.,yaw=0.,height=1.)],
             targets=[dict(id=7,x=4.,y=1.3,yaw=math.pi,size=.3)])
    env=SearchEnv(cfg,seed=0)
    env.rotate(2*math.pi)
    assert 7 not in env.found
    assert env.map[int(1.3/env.res),int(5./env.res)]<0


def test_training_regression_no_collision_from_rounding():
    cfg=make_scenario(1)
    a=SearchEnv(cfg,seed=123457).run(BASELINE)
    b=SearchEnv(cfg,seed=123457).run(BASELINE)
    assert a==b
    assert a['collisions']==0
    assert a['seconds']<=600


def test_hidden_targets_are_on_distinct_obstacles_and_not_visible_at_start():
    for seed in [0,1,2,3]:
        cfg=make_scenario(seed)
        assert cfg['task']=='hidden_obstacle_aruco_v2'
        assert len({t['carrier_obstacle'] for t in cfg['targets']})==3
        for target in cfg['targets']:
            assert target['hidden_from_start']
            dx=cfg['start'][0]-target['x']; dy=cfg['start'][1]-target['y']
            assert dx*math.cos(target['yaw'])+dy*math.sin(target['yaw'])<0
        env=SearchEnv(cfg,seed=0); env.rotate(2*math.pi)
        assert not env.found


def test_sparse_slam_cells_do_not_trap_robot_in_endless_spin():
    # Unknown scan gaps are not occupied obstacles. Nav2 still checks its footprint.
    data=np.zeros((50,50),dtype=np.int8)
    data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    data[13,13:18]=-1; data[17,13:18]=-1
    data[13:18,13]=-1; data[13:18,17]=-1; data[15,13]=0
    choices,_=options(data,.1,(0,0),(1.55,1.55))
    assert choices
    assert choose(choices,BASELINE)['distance']>.4
