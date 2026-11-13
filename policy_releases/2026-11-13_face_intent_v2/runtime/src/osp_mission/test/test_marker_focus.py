import math

import numpy as np

from osp_mission.marker_focus import MarkerFocus


def hint(focus, stamp=10., ident=42, point=(5.,4.), observer=(5.,2.)):
    focus.observe(dict(stamp=stamp,id=ident,map_point=[*point,.5],observation_frame='map'),observer,stamp)


def choose(focus, now=12., confirmed=(), robot=(5.,2.), candidates=(), data=None):
    if data is None:data=np.zeros((100,100),dtype=np.int8)
    return focus.choose(data,.1,(0.,0.),robot,0.,list(candidates),confirmed,now)


def test_accepted_single_detection_can_prompt_stationary_look_without_confirming_it():
    focus=MarkerFocus();hint(focus)
    selected=choose(focus)
    assert selected['focus_id']==42 and selected['stationary']
    assert math.isclose(selected['scan_angle'],math.pi/2)
    assert focus.hints[42]['count']==1


def test_confirmed_or_three_observations_do_not_interrupt_coverage():
    focus=MarkerFocus();hint(focus)
    assert choose(focus,confirmed={42:(5.,4.)}) is None
    hint(focus,stamp=10.2);hint(focus,stamp=10.4)
    assert choose(focus) is None


def test_recent_publication_stale_wrong_frame_and_repeated_stamp_are_excluded():
    focus=MarkerFocus();hint(focus);hint(focus)
    assert focus.hints[42]['count']==1
    assert choose(focus,now=10.5) is None
    assert choose(focus,now=611.) is None
    focus.observe(dict(stamp=11.,id=9,map_point=[5.,4.,.5],observation_frame='odom'),(5.,2.),11.)
    focus.observe(dict(stamp=9.,id=8,map_point=[5.,4.,.5],observation_frame='map'),(5.,2.),11.)
    assert set(focus.hints)=={42}


def test_followups_have_cooldown_and_four_attempt_limit():
    focus=MarkerFocus();hint(focus)
    assert choose(focus,now=12.) is not None
    assert choose(focus,now=13.) is None
    assert choose(focus,now=42.) is not None
    assert choose(focus,now=72.) is not None
    assert choose(focus,now=102.) is not None
    assert choose(focus,now=132.) is None


def test_unconfirmed_hint_remains_revisitable_after_ninety_seconds():
    focus=MarkerFocus();hint(focus)
    assert choose(focus,now=150.) is not None
    assert choose(focus,now=611.) is None


def test_occlusion_and_opposite_side_are_not_selected():
    focus=MarkerFocus();hint(focus)
    assert choose(focus,robot=(5.,6.)) is None
    data=np.zeros((100,100),dtype=np.int8);data[30,40:60]=100
    assert choose(focus,data=data) is None


def test_far_observation_can_select_closer_safe_waypoint_aimed_at_marker():
    focus=MarkerFocus();hint(focus,observer=(5.,0.))
    candidate=dict(x=5.,y=2.,distance=2.,features=np.array([1.,0.,2.,0.,0.]))
    selected=choose(focus,robot=(5.,0.),candidates=[candidate])
    assert selected is not None and not selected.get('stationary')
    assert selected['scan_angle']==0. and math.isclose(selected['yaw'],math.pi/2)
    assert candidate['features'][3]==0.  # original policy candidate remains unchanged


def test_accepted_image_can_add_viewpoint_missing_from_sparse_exploration_grid():
    focus=MarkerFocus();hint(focus,point=(5.,6.),observer=(5.,1.))
    selected=choose(focus,robot=(5.,1.))
    assert selected is not None and selected['targeted']
    assert selected['distance']<=4.
    assert math.dist((selected['x'],selected['y']),(5.,6.))<=2.3+1e-9


def test_targeted_view_requires_clearance_along_the_entire_direct_route():
    focus=MarkerFocus();hint(focus,point=(5.,6.),observer=(5.,1.))
    data=np.zeros((100,100),dtype=np.int8);data[25,:]=100
    assert choose(focus,robot=(5.,1.),data=data) is None


def test_targeted_view_does_not_enter_unknown_space():
    focus=MarkerFocus();hint(focus,point=(5.,6.),observer=(5.,1.))
    data=np.zeros((100,100),dtype=np.int8);data[25:,:]=-1
    assert choose(focus,robot=(5.,1.),data=data) is None


def test_navigation_interrupt_requires_fresh_unconfirmed_image_and_is_bounded():
    focus=MarkerFocus();hint(focus)
    assert not focus.request_interrupt(42,{42},10.1)
    assert not focus.request_interrupt(42,{},11.)
    assert focus.request_interrupt(42,{},10.2)
    assert not focus.request_interrupt(42,{},10.3)
    for ident in range(5):
        hint(focus,ident=ident)
        assert focus.request_interrupt(ident,{},10.2)
    hint(focus,ident=99)
    assert not focus.request_interrupt(99,{},10.2)


def test_three_accepted_frames_do_not_interrupt_navigation():
    focus=MarkerFocus();hint(focus);hint(focus,stamp=10.2);hint(focus,stamp=10.4)
    assert not focus.request_interrupt(42,{},10.5)


def test_live_hint_callback_pauses_nav_without_counting_an_unconfirmed_target():
    import json
    from types import SimpleNamespace
    from std_msgs.msg import String
    from osp_mission.mission import Mission
    canceled=[];stopped=[];stamps=[]
    def lookup(map_frame,base_frame,stamp):
        stamps.append(stamp.nanoseconds)
        return SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(x=5.,y=2.)))
    owner=SimpleNamespace(marker_focus=MarkerFocus(),tf=SimpleNamespace(lookup_transform=lookup),
        p=lambda key:'map' if key=='map_frame' else 'base_link',now=lambda:10.2,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000)),
        state='running',kind='nav',goal_handle=object(),pending=False,cancel_reason='',targets={},
        cancel_action=canceled.append,zero=lambda:stopped.append(True),
        get_logger=lambda:SimpleNamespace(info=lambda _:None,debug=lambda _:None))
    msg=String(data=json.dumps(dict(id=42,stamp=10.,map_point=[5.,4.,.5],observation_frame='map')))
    Mission.on_marker_hint(owner,msg)
    assert stamps==[10_000_000_000]
    assert canceled==['marker_hint'] and stopped and owner.targets=={}
    Mission.on_marker_hint(owner,msg)
    assert canceled==['marker_hint']


def test_hint_cancellation_does_not_blacklist_the_unfinished_goal_or_trigger_recovery():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from osp_mission.mission import Mission
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='nav',
        cancel_reason='marker_hint',excluded=[],destination=(1.,2.),scan_after=True,
        recovery_pending=False,now=lambda:100.)
    result=Future();result.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,result)
    assert owner.goal_handle is None and owner.excluded==[]
    assert not owner.scan_after and not owner.recovery_pending and owner.plan_after==101.


def test_hint_carries_exposure_time_camera_origin_without_waiting_for_another_tf():
    import json
    from types import SimpleNamespace
    from std_msgs.msg import String
    from osp_mission.mission import Mission
    owner=SimpleNamespace(marker_focus=MarkerFocus(),tf=None,p=lambda key:'map',now=lambda:10.2,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000)),
        state='running',kind='spin',goal_handle=object(),pending=False,cancel_reason='',targets={},
        get_logger=lambda:SimpleNamespace(debug=lambda _:None))
    message=String(data=json.dumps(dict(id=42,stamp=10.,map_point=[5.,4.,.5],
        camera_origin_map=[5.,2.,.24],observation_frame='map')))
    Mission.on_marker_hint(owner,message)
    assert owner.marker_focus.hints[42]['observer'].tolist()==[5.,2.]
    assert owner.marker_focus.hints[42]['count']==1
