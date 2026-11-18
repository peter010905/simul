from osp_mission.contact_inspection import ContactInspection, BearingCue


def test_failed_retreat_requires_completed_turn_before_any_navigation():
    r=ContactInspection()
    assert r.hazard(['bump_left'],10)
    assert r.recovery(10.1, True, True, True)=='hold'
    r.hazard([],10.2)
    assert r.recovery(10.8, True, True, True)=='backup'
    r.action='backup'
    r.result(False,11)
    r.hazard([],11.2)
    assert r.recovery(11.2,False,True,True)=='spin' and r.side<0
    r.action='spin'
    r.result(False,11.3)
    assert r.recovery(12,True,True,True)=='hold'
    r.hazard([],14)
    assert r.recovery(14,False,True,True)=='spin'
    r.action='spin'
    r.result(True,15)
    assert r.recovery(15,False,True,True)==''


def test_stale_hazard_and_recontact_hold_motion():
    r=ContactInspection()
    r.hazard(['bump_right'],0)
    r.hazard([],1)
    assert r.recovery(3,True,True,True)=='hold'
    r.hazard([],3.1)
    assert r.recovery(3.1,False,True,True)=='spin' and r.side>0
    r.action='spin'
    assert r.hazard(['bump_left'],3.3)
    r.result(False,3.4)
    assert r.recovery(3.5,True,True,True)=='hold'


def test_camera_search_requires_elapsed_time_and_displacement():
    r=ContactInspection()
    r.position=(0.,0.)
    r.looked(1.)
    r.position=(3.,0.)
    assert not r.due(20.) and r.due(27.)
    assert r.survey_angle()==-r.survey_angle()


def test_bearing_only_uses_distinct_coherent_frames_and_cooldown():
    cue=BearingCue()
    assert not cue.observe(42,1.,.35)
    assert not cue.observe(42,1.,.35)
    assert not cue.observe(42,1.1,.33)
    assert cue.observe(42,1.2,.34)
    assert not cue.observe(42,1.3,.33)
