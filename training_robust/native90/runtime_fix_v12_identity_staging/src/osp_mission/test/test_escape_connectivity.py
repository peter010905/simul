import numpy as np
from scipy.ndimage import distance_transform_edt, label

from osp_mission.escape_connectivity import connect_start
from osp_mission.edge_policy import options


def test_near_wall_start_rejoins_safe_region_without_near_wall_goals():
    data=np.zeros((60,60),dtype=int);data[:,0]=100
    clearance=distance_transform_edt(data<=50)*.05
    safe=(data==0)&(clearance>=.32)
    connected=connect_start(data,.05,(30,5),safe)
    groups,_=label(connected)
    assert connected[30,5] and groups[30,5]==groups[30,20]
    candidates,_=options(data,.05,(0,0),(.275,1.525))
    assert candidates
    assert all(clearance[c['cell']]>=.32 for c in candidates)


def test_unknown_or_insufficient_body_clearance_is_not_opened():
    data=np.zeros((40,40),dtype=int);data[:,:10]=-1
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    # Unknown start must never become traversable.
    assert not connect_start(data,.05,(20,9),safe)[20,9]
    data[:,10]=100
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    assert not connect_start(data,.05,(20,13),safe)[20,13]


def test_escape_cannot_open_long_narrow_corridor():
    data=np.zeros((60,60),dtype=int);data[:,:20]=100;data[:,30:]=100
    data[50:,20:50]=0
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    assert not connect_start(data,.05,(10,25),safe)[10,25]
