import math

import numpy as np

from osp_mission.near_pass_inspection import NearPassInspection


def scene():
    grid=np.zeros((100,100),dtype=np.int8);grid[60,20:80]=100
    viewed=np.zeros((4,)+grid.shape,dtype=bool)
    return grid,viewed


def choose(policy=None,**kwargs):
    policy=policy or NearPassInspection();grid,viewed=scene()
    args=dict(data=grid,resolution=.1,origin=(0.,0.),camera=(5.,5.,0.),robot=(5.,5.),
        viewed=viewed,focal=665.,fov=1.25,odom_camera=(1.,2.,0.),now=100.,elapsed=100.,remaining=500.)
    args.update(kwargs);return policy,policy.choose(**args)


def test_side_face_gets_bounded_look_and_current_field_of_view_does_not():
    policy,goal=choose()
    assert goal and 0<goal['turn_rad']<=2*math.pi/3
    assert goal['range_max_m']<=1.2 and policy.report()['requested']==0
    assert choose(camera=(5.,5.,math.pi/2))[1] is None


def test_unknown_or_blocked_footprint_is_not_used_for_a_stationary_turn():
    grid,viewed=scene();grid[50,50]=-1
    assert choose(data=grid,viewed=viewed)[1] is None
    grid[50,50]=0;grid[50,52]=100
    assert choose(data=grid,viewed=viewed)[1] is None


def test_viewed_far_or_back_faces_create_no_glance():
    grid,viewed=scene();viewed[:]=True
    assert choose(viewed=viewed)[1] is None
    assert choose(camera=(5.,4.,0.),robot=(5.,4.))[1] is None
    assert choose(camera=(5.,7.,0.),robot=(5.,7.),viewed=viewed)[1] is None


def test_marker_navigation_deadline_and_initial_survey_are_protected():
    assert choose(protected=True)[1] is None
    assert choose(remaining=40.)[1] is None
    assert choose(elapsed=10.)[1] is None


def test_reservation_cooldown_and_odometry_memory_prevent_repeated_looks():
    policy,goal=choose();policy.reserve(goal,100.)
    assert choose(policy,now=101.)[1] is None
    policy.finish(104.,'three_processed_frames')
    assert choose(policy,now=105.)[1] is None
    assert choose(policy,now=200.)[1] is None
    # A different physical viewpoint remains eligible after a map correction.
    assert choose(policy,now=200.,odom_camera=(4.,2.,0.))[1] is not None


def test_actual_time_budget_and_request_count_bound_additional_actions():
    policy=NearPassInspection();policy.spent=35.
    assert choose(policy,now=590.,elapsed=590.)[1] is None
    policy=NearPassInspection();policy.history=[{}]*8
    assert choose(policy)[1] is None


def test_rejection_or_sensor_interruption_does_not_create_any_map_or_id_credit():
    policy,goal=choose();policy.reserve(goal,100.);policy.finish(108.,'spin_blocked')
    report=policy.report();assert not report['active'] and report['spent_sim_s']==8.
    assert not report['creates_recognition_or_coverage_credit']
