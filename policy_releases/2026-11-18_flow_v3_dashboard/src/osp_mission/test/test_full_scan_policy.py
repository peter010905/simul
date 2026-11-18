import math

import numpy as np

from osp_mission.edge_policy import options


def test_full_scan_counts_all_visible_faces_and_keeps_same_safe_viewpoints():
    data=np.zeros((70,70),dtype=np.int8)
    data[5:65,5]=data[5:65,64]=100
    data[5,5:65]=data[64,5:65]=100
    data[25:35,34:37]=100
    partial,_=options(data,.1,(0.,0.),(3.,4.),collect_faces=True)
    full,_=options(data,.1,(0.,0.),(3.,4.),collect_faces=True,full_scans=True)
    assert partial and len(full)==len(partial)
    assert [c['cell'] for c in partial]==[c['cell'] for c in full]
    assert any(a['features'][0]<b['features'][0] for a,b in zip(partial,full))
    for a,b in zip(partial,full):
        assert b['scan_angle']==2*math.pi
        assert b['features'][0]>=a['features'][0]
        assert math.isclose(b['features'][0],len(b['visible_faces'])*.1)
        assert math.isclose(b['yaw'],math.atan2(b['y']-4.,b['x']-3.))
