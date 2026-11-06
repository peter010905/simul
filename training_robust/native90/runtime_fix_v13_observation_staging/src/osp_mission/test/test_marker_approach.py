import numpy as np
import pytest

from osp_mission.marker_approach import MarkerApproach
from osp_mission.native_audit import NativeAudit


def observe(tracker, ident, distance, stamp=1.):
    tracker.observe(dict(id=ident, stamp=stamp, map_point=[3.,3.], observation_frame='map'),
                    [3.-distance,3.], stamp+.1)


def test_far_decoded_ids_cannot_finish():
    tracker=MarkerApproach()
    for ident in (1,2,3):
        for t in (1.,1.2,1.4):observe(tracker,ident,2.5,t)
    assert not tracker.complete({1:(3,3),2:(3,3),3:(3,3)})


def test_confirmed_ids_and_independent_close_pose_finish_without_near_rgb():
    tracker=MarkerApproach()
    targets={1:(3,3),2:(3,3),3:(3,3)}
    for ident in targets:
        for t in (1.,1.2,1.4):observe(tracker,ident,1.2,t)
    assert not tracker.complete(targets)
    tracker.observe_pose(targets,(1.5,3.),2.,2.)
    assert tracker.complete(targets)
    assert not tracker.complete({1:(3,3),2:(3,3),4:(3,3)})
    assert not tracker.complete({1:(3,3),2:(3,3)})


def test_stale_pose_and_target_correction_do_not_credit_approach():
    tracker=MarkerApproach();targets={1:(3.,3.)}
    tracker.observe_pose(targets,(2.,3.),1.,3.)
    assert not tracker.verified(1)
    tracker.observe_pose(targets,(2.,3.),3.,3.)
    assert tracker.verified(1)
    assert not tracker.verified(1,(4.,3.))
    assert not tracker.verified(2)


def test_duplicate_or_stale_frames_do_not_confirm():
    tracker=MarkerApproach()
    for _ in range(5):observe(tracker,1,1.2)
    assert not tracker.verified(1)
    tracker.observe(dict(id=1,stamp=2.,map_point=[3.,3.],observation_frame='map'),[2.,3.],4.)
    assert tracker.hints[1]['stamp']==1.


def test_close_viewpoint_and_blocked_shortcut():
    tracker=MarkerApproach();observe(tracker,1,3.)
    grid=np.zeros((80,80),dtype=int)
    result=tracker.choose(grid,.1,(0,0),(1.,3.),0.,[],{1:(3.,3.)},10.)
    assert result is not None and np.linalg.norm(np.array([result['x'],result['y']])-[3.,3.])<=1.2
    grid[:,15:18]=100
    tracker.last_attempt.clear()
    assert tracker.choose(grid,.1,(0,0),(1.,3.),0.,[],{1:(3.,3.)},30.) is None


@pytest.mark.parametrize('distance,expected',[(1.49,True),(1.5,True),(1.51,False),(2.,False)])
def test_evaluator_enforces_actual_one_point_five_metres(tmp_path,distance,expected):
    audit=NativeAudit({'targets':[{'id':1,'x':3.,'y':3.}]})
    audit.calibrate((0,0,0),(0,0,0));audit.observe(1.,3.-distance,3.,0.)
    path=tmp_path/'markers.csv';path.write_text('id,x,y\n1,3,3\n')
    result=audit.evaluate(path)
    assert result['all_approached']==expected
    assert result['correct_ids'] and result['correct_positions']
