import math

import numpy as np

from osp_mission.edge_policy import EdgeCoverage,options,readable_distance


def wall_options(focal,x):
    grid=np.zeros((100,100),dtype=np.int8);grid[20:81,70]=100
    goals,_=options(grid,.1,(0.,0.),(2.,5.),inspection_focal=focal,collect_faces=True,
        point_sampler=lambda *args:[(50,x)])
    return grid,goals


def test_far_wall_that_cannot_be_inspected_gets_no_camera_gain():
    grid,legacy=wall_options(None,39)
    assert legacy and legacy[0]['features'][0]>0
    _,calibrated=wall_options(665.29501224,39)
    assert not calibrated  # 3.05m gives <18px at 8cm, outside the 2.513m conservative range.


def test_near_rewarded_faces_are_actually_confirmable_in_camera_frames():
    focal=665.29501224;grid,goals=wall_options(focal,50)
    assert goals and 0<goals[0]['scan_angle']<=2*math.pi/3
    goal=goals[0];coverage=EdgeCoverage()
    points=np.array([[7.05,(y+.5)*.1] for y in range(20,81)])
    center=goal['yaw']+math.pi/3
    for stamp in [1.,1.2,1.4]:
        coverage.observe(grid,.1,(0.,0.),(goal['x'],goal['y'],center),points,stamp,focal,1.25)
    face=goal['visible_faces'];assert len(face)>0
    assert np.any(coverage.viewed[face[:,0],face[:,1],face[:,2]])


def test_actual_focal_length_changes_which_wall_views_are_rewarded():
    assert not wall_options(500.,45)[1]
    assert wall_options(665.29501224,45)[1]
    assert readable_distance(665.29501224)>readable_distance(500.)


def test_unavailable_or_invalid_native_calibration_cannot_invent_surface_reward():
    for focal in [0.,-1.,float('nan')]:
        assert readable_distance(focal)==0.
        assert not wall_options(focal,50)[1]
