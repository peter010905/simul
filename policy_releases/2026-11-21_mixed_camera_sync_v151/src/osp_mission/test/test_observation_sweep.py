"""Bounded inspection observes new faces without repeating full rotations."""
import math
import numpy as np
import pytest
from osp_mission.observation_sweep import select


def scene(side):
    data=np.zeros((60,60),dtype=np.int8)
    data[40 if side=='left' else 20,5:55]=100
    return data,np.zeros((4,)+data.shape,dtype=bool)


@pytest.mark.parametrize('side,sign',[('left',1),('right',-1)])
def test_shortest_direction_with_hard_120_degree_cap(side,sign):
    data,viewed=scene(side)
    angle=select(data,.1,(0,0),(3.,3.,0.),viewed,665.,1.25)
    assert 0<sign*angle<=2*math.pi/3
    assert abs(angle)<2*math.pi/3  # the wall fits a shorter arc


def test_all_inspected_or_no_surfaces_skips_turn():
    data,viewed=scene('left');viewed[:]=True
    assert select(data,.1,(0,0),(3.,3.,0.),viewed,665.,1.25)==0.
    assert select(np.zeros_like(data),.1,(0,0),(3.,3.,0.),viewed,665.,1.25)==0.


def test_occluded_far_and_already_forward_faces_do_not_trigger_spin():
    data,viewed=scene('left')
    forward=np.zeros_like(data);forward[40,27:33]=100
    assert select(forward,.1,(0,0),(3.,3.,math.pi/2),viewed,665.,1.25)==0.
    # Mark the nearer blocker inspected: only the hidden wall is unseen.
    data[35,:]=100;viewed[:,35,:]=True
    assert select(data,.1,(0,0),(3.,3.,0.),viewed,665.,1.25)==0.
    data,viewed=scene('left')
    assert select(data,.1,(0,0),(3.,.5,0.),viewed,100.,1.25)==0.


def test_invalid_camera_and_small_requested_cap():
    data,viewed=scene('left')
    assert select(data,.1,(0,0),(3.,3.,0.),viewed,float('nan'),1.25)==0.
    angle=select(data,.1,(0,0),(3.,3.,0.),viewed,665.,1.25,maximum=.3)
    assert abs(angle)<=.3



def test_selection_never_credits_surfaces_without_camera_observations():
    data,viewed=scene('left');before=viewed.copy()
    select(data,.1,(0,0),(3.,3.,0.),viewed,665.,1.25)
    np.testing.assert_array_equal(viewed,before)
