import math

import numpy as np
import pytest

from osp_mission.marker_approach import MarkerApproach


def fixture():
    data=np.zeros((70,100),dtype=np.int8);data[:2]=100;data[-2:]=100;data[:,:2]=100;data[:,-2:]=100
    data[20:,49:52]=100;data[33:38,80:83]=100
    tracker=MarkerApproach();tracker.hints[44]=dict(point=np.array([8.,3.5]),observer=np.array([2.,3.5]),stamp=1.)
    return data,tracker


def choose(data,tracker):return tracker.choose(data,.1,(0.,0.),(2.,3.5),0.,[],{44:(8.,3.5)},10.)


def test_reachable_viewpoint_around_partition_uses_route_length_and_stays_on_observed_side():
    data,tracker=fixture();goal=choose(data,tracker)
    assert goal is not None and goal['approach_id']==44
    assert goal['x']<8. and .65<=math.hypot(goal['x']-8.,goal['y']-3.5)<=.9
    assert goal['distance']>math.hypot(goal['x']-2.,goal['y']-3.5)+.3
    assert tracker.report()['verified_ids']==[]


@pytest.mark.parametrize('fill',[100,-1])
def test_disconnected_or_unknown_corridor_is_never_a_navigation_route(fill):
    data,tracker=fixture();data[:,49:52]=fill
    assert choose(data,tracker) is None


def test_unknown_space_around_marker_cannot_create_a_close_viewpoint():
    data,tracker=fixture();data[:,65:79]=-1
    assert choose(data,tracker) is None


def test_wrong_observed_side_cannot_create_a_shortcut_through_the_carrier():
    data,tracker=fixture();tracker.hints[44]['observer']=np.array([9.,3.5])
    assert choose(data,tracker) is None


def test_existing_stationary_close_inspection_survives_when_no_translation_route_can_start():
    data=np.zeros((70,100),dtype=np.int8);data[:2]=100;data[-2:]=100;data[:,:2]=100;data[:,-2:]=100
    data[35,18]=100
    tracker=MarkerApproach();tracker.hints[44]=dict(observer=np.array([2.,3.5]),point=np.array([3.,3.5]),stamp=1.)
    goal=tracker.choose(data,.1,(0.,0.),(2.,3.5),0.,[],{44:(3.,3.5)},10.)
    assert goal is not None and goal.get('stationary')
