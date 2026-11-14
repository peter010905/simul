import math
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt,label
from osp_mission.escape_connectivity import connect_start
from osp_mission.edge_policy import options


def pocket():
    # 55 cm wide, 3.25 m exit route into a wide room. TB4 diameter is 37.8 cm.
    data=np.full((120,160),100,dtype=np.int16)
    data[15:105,85:150]=0
    data[55:66,10:95]=0
    return data,(60,20)


def test_long_body_clear_exit_is_not_mistaken_for_blockage():
    data,start=pocket();resolution=.05
    clearance=distance_transform_edt(data<=50)*resolution
    safe=(data==0)&(clearance>=.32)
    assert not connect_start(data,resolution,start,safe,max_escape=.8)[start]
    connected=connect_start(data,resolution,start,safe)
    groups,_=label(connected)
    assert connected[start] and groups[start]==groups[60,120]
    added=connected&~safe
    body=distance_transform_edt(np.pad(data==0,1,constant_values=False))[1:-1,1:-1]*resolution
    assert np.all(data[added]==0)
    assert np.min(body[added])>=.189+resolution/math.sqrt(2)+.01
    candidates,previous=options(data,resolution,(0,0),((start[1]+.5)*resolution,(start[0]+.5)*resolution),return_paths=True,inspection_focal=308.)
    assert candidates and previous is not None
    assert all(clearance[c['cell']]>=.32 for c in candidates)


@pytest.mark.parametrize('barrier',[-1,100])
def test_long_exit_does_not_cut_unknown_or_occupied_barrier(barrier):
    data,start=pocket();data[55:66,50:52]=barrier
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    assert not connect_start(data,.05,start,safe)[start]
    assert not options(data,.05,(0,0),((start[1]+.5)*.05,(start[0]+.5)*.05))[0]


def test_too_narrow_exit_remains_blocked():
    data,start=pocket();data[55:58,10:85]=100;data[63:66,10:85]=100
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    assert not connect_start(data,.05,start,safe)[start]


def test_closed_pocket_without_room_is_not_opened():
    data,start=pocket();data[:,85:]=100
    safe=(data==0)&(distance_transform_edt(data<=50)*.05>=.32)
    assert np.array_equal(connect_start(data,.05,start,safe),safe)
