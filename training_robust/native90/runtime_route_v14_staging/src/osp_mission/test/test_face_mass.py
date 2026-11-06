"""Connected physical sides retain distinct, bounded inspection rewards."""

import numpy as np
from osp_mission.face_mass import weights


def test_long_face_capped_but_small_face_and_opposite_side_preserved():
    data=np.zeros((100,120),dtype=np.int8)
    data[10:90,30]=100;data[20:28,70:78]=100
    mass=weights(data,.1,1.5)
    assert np.isclose(mass[3,10:90,30].sum()*.1,1.5)
    assert np.isclose(mass[1,10:90,30].sum()*.1,1.5)
    assert np.all(mass[3,20:28,70]==1.)
    rotated=weights(np.rot90(data).copy(),.1,1.5)
    for side in range(4):np.testing.assert_allclose(np.rot90(mass[side]),rotated[(side-1)%4])
