import numpy as np
from scipy.ndimage import distance_transform_edt
from osp_mission.edge_policy import options


def test_unseen_obstacle_faces_receive_safe_close_observation_candidates():
    data=np.zeros((80,100),dtype=int)
    data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    data[25:55,45:50]=100
    choices,_=options(data,.1,(0,0),(2.,4.))
    close=[c for c in choices if c.get('face_inspection')]
    assert close and len(close)<=96
    clearance=distance_transform_edt(data<=50)*.1
    assert all(data[c['cell']]==0 and clearance[c['cell']]>=.32 for c in close)
    assert any(c['x']<4.5 for c in close) and any(c['x']>5 for c in close)


def test_fully_observed_faces_do_not_generate_inspection_candidates():
    data=np.zeros((40,40),dtype=int);data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    choices,_=options(data,.1,(0,0),(2.,2.),viewed=np.ones((4,40,40),dtype=bool))
    assert not any(c.get('face_inspection') for c in choices)
