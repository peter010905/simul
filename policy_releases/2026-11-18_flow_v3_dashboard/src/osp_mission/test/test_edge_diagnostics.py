"""Observation-only diagnostics must not change policy options or scoring."""

import numpy as np

from osp_mission.edge_policy import choose, options


def test_face_diagnostics_preserve_candidate_order_and_scores():
    data=np.zeros((50,70),dtype=np.int8)
    data[0,:]=data[-1,:]=data[:,0]=data[:,-1]=100
    data[15:35,35]=100
    normal,_=options(data,.1,(0.,0.),(1.,1.))
    detailed,_=options(data,.1,(0.,0.),(1.,1.),collect_faces=True)
    assert normal and len(normal)==len(detailed)
    for a,b in zip(normal,detailed):
        for key,value in a.items():
            np.testing.assert_array_equal(value,b[key])
        faces=b['visible_faces']
        assert len(faces)*.1==b['features'][0]
        assert np.all(data[faces[:,1],faces[:,2]]>50)
    assert choose(normal)['cell']==choose(detailed)['cell']
