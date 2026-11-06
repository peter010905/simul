import math
from xml.dom.minidom import parseString
import numpy as np
from osp_mission.assignment_scenarios import make_assignment
from osp_mission.scenarios import rasterize,valid
from osp_mission.sensor_model import configure_sensors
from osp_mission.inspection_policy import options,choose
from osp_mission.fast_sim import SearchEnv


def test_all_four_walls_survive_rasterization():
    for resolution in [.05,.15,.20]:
        occ=rasterize(dict(obstacles=[],targets=[]),resolution)
        assert occ[0].all() and occ[-1].all()
        assert occ[:,0].all() and occ[:,-1].all()


def test_assignment_has_accessible_mixed_vertical_mounts():
    for seed,count in [(1,4),(2,6),(3,8)]:
        cfg=make_assignment(seed,count,True)
        assert cfg==make_assignment(seed,count,True) and valid(cfg)
        assert len(cfg['obstacles'])==count and len(cfg['targets'])==3
        assert len({t['id'] for t in cfg['targets']})==3
        assert all(o['height']>=.9 for o in cfg['obstacles'])


def test_rgb_and_depth_share_configured_sensor_dimensions():
    doc=parseString('<robot><sensor type="rgbd_camera"><update_rate>30</update_rate><camera><horizontal_fov>1.25</horizontal_fov><image><width>320</width><height>240</height></image></camera></sensor></robot>')
    configure_sensors(doc,960)
    assert doc.getElementsByTagName('width')[0].firstChild.data=='960'
    assert doc.getElementsByTagName('height')[0].firstChild.data=='720'
    assert doc.getElementsByTagName('update_rate')[0].firstChild.data=='5'
    assert doc.getElementsByTagName('horizontal_fov')[0].firstChild.data=='1.25'


def test_inspection_goals_stay_in_observed_connected_free_space():
    data=np.full((60,80),-1,dtype=np.int8);data[5:55,5:65]=0;data[5:55,30]=100
    choices,_=options(data,.1,(0,0),(1.,1.))
    assert choices and all(c['x']<3. for c in choices)
    assert data[choose(choices)['cell']]==0


def test_surrogate_requires_temporally_separated_confirmations():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[dict(id=7,x=3.,y=2.,yaw=math.pi,size=.3)])
    env=SearchEnv(cfg,seed=1,camera_width=960)
    class PerfectDetection:
        def random(self):return 1.
    env.rng=PerfectDetection();env.found.clear();env.confirmations.clear();env.last_confirmation.clear()
    for _ in range(5):env.detect()
    assert not env.found
    env.elapsed=.2;env.detect();assert not env.found
    env.elapsed=.4;env.detect();assert env.found=={7}
