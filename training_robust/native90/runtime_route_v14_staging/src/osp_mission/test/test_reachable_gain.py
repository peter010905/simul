"""Unknown space behind a completed room wall is not an exploration frontier."""
import numpy as np
import pytest

from osp_mission import directional_policy, edge_policy, surface_endpoint_policy


@pytest.mark.parametrize('module,sides', [(surface_endpoint_policy,0),(directional_policy,8),(edge_policy,4)])
def test_enclosed_observed_room_does_not_reward_unreachable_exterior(module,sides):
    data=np.full((80,100),-1,dtype=np.int8)
    data[10:70,10:90]=0
    data[10:70,10]=data[10:70,89]=100
    data[10,10:90]=data[69,10:90]=100
    viewed=np.ones((sides,)+data.shape if sides else data.shape,dtype=bool)
    choices,_=module.options(data,.1,(0.,0.),(5.,4.),viewed=viewed)
    assert not choices
    # An open exit leading to unknown space still attracts exploration.
    data[10,43:57]=0
    choices,_=module.options(data,.1,(0.,0.),(5.,4.),viewed=viewed)
    assert choices and max(g['features'][1] for g in choices) > 0
    assert all(g['y']<3 for g in choices)
