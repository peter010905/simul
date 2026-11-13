"""Observed pixels influence viewpoint costs without changing evidence credit."""
import math

import numpy as np
import pytest

from osp_mission.marker_focus import MarkerFocus


def choice(pixels,*,data=None,confirmed=(),metadata=None):
    focus=MarkerFocus();evidence=dict(stamp=10.,id=42,map_point=[5.,4.,.5],
        observation_frame='map',min_edge_pixels=pixels,**(metadata or {}))
    focus.observe(evidence,(5.,1.9),10.)
    if data is None:data=np.zeros((100,100),dtype=np.int8)
    selected=focus.choose(data,.1,(0.,0.),(5.,1.9),math.pi/2,[],confirmed,12.)
    return focus,selected


@pytest.mark.parametrize('pixels',[10.,22.2,28.])
def test_fragile_glimpse_prefers_a_closer_safe_view_with_larger_estimated_marker(pixels):
    focus,selected=choice(pixels)
    assert selected and not selected.get('stationary')
    assert math.dist((selected['x'],selected['y']),(5.,4.))<2.1
    assert selected['predicted_min_edge_pixels']>pixels
    assert focus.hints[42]['count']==1  # changing the view never confirms an ID


@pytest.mark.parametrize('pixels',[36.,80.])
def test_large_readable_glimpse_keeps_fast_stationary_confirmation(pixels):
    _,selected=choice(pixels)
    assert selected['stationary'] and selected['view_resolution_penalty_s']==0


@pytest.mark.parametrize('pixels',[None,float('nan'),'invalid'])
def test_missing_image_measurement_does_not_invent_a_pixel_estimate(pixels):
    _,selected=choice(pixels)
    assert selected['stationary'] and selected['predicted_min_edge_pixels'] is None


def test_no_safe_closer_endpoint_keeps_stationary_fallback():
    data=np.full((100,100),100,dtype=np.int8);data[:,48:52]=0
    _,selected=choice(22.,data=data)
    assert selected['stationary']


def test_unknown_space_and_confirmed_identity_are_not_overridden_by_pixel_incentive():
    data=np.zeros((100,100),dtype=np.int8);data[23:,:]=-1
    _,selected=choice(10.,data=data)
    assert selected is None
    _,selected=choice(10.,confirmed={42:(5.,4.)})
    assert selected is None


def test_true_marker_size_metadata_is_ignored():
    _,small=choice(22.2,metadata={'marker_size_m':.01})
    _,large=choice(22.2,metadata={'marker_size_m':10.})
    assert (small['x'],small['y'],small['yaw'])==(large['x'],large['y'],large['yaw'])
