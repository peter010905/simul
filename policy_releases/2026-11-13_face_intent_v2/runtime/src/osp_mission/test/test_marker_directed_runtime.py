import math
from concurrent.futures import Future
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from osp_mission.marker_directed_look import MarkerDirectedLook
from osp_mission.mission import Mission


def owner(camera=(0.,0.,0.)):
    x,y,yaw=camera
    # Optical Z faces forward, optical X right, optical Y down.
    matrix=np.array([[math.sin(yaw),0.,math.cos(yaw)],
                     [-math.cos(yaw),0.,math.sin(yaw)],[0.,-1.,0.]])
    q=Rotation.from_matrix(matrix).as_quat()
    transform=NS(transform=NS(translation=NS(x=x,y=y),rotation=NS(**dict(zip(('x','y','z','w'),q)))))
    log=NS(info=lambda *a:None,debug=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
    o=NS(marker_focus=NS(hints={292:dict(point=np.array([2.,0.]))}),targets={440:np.array([0.,2.])},
         tf=NS(lookup_transform=lambda target,source,stamp:transform),inspection_camera_frame='rgb_optical',
         now=lambda:100.,get_logger=lambda:log,edge_mode=True,map=None)
    o.marker_look=MarkerDirectedLook(o);o.events=[]
    o.marker_look.event=lambda event,**kw:o.events.append(dict(event=event,**kw))
    return o


@pytest.mark.parametrize('stationary',[True,False])
def test_marker_look_bypasses_generic_sweep_for_stationary_and_arrived_goals(monkeypatch,stationary):
    o=owner((0.,0.,-.09805284696110128))
    o.marker_look.select(dict(focus_id=292,scan_angle=.09805284696110128,stationary=stationary),(0.,0.,0.))
    monkeypatch.setattr('osp_mission.mission.select_observation_sweep',lambda *a:pytest.fail('generic sweep replaced marker'))
    assert Mission.observation_turn(o)==pytest.approx(.09805284696110128)
    assert o.marker_look.pending is None
    assert o.marker_look.report()['executed']==1
    assert not o.marker_look.report()['creates_recognition_or_approach_credit']


def test_actual_camera_pose_includes_lever_arm_and_optical_heading():
    o=owner((.06,-.03,.2));o.marker_look.select(dict(focus_id=292),(0.,0.,0.))
    assert Mission.observation_turn(o)==pytest.approx(math.atan2(.03,1.94)-.2)


@pytest.mark.parametrize('yaw',[math.pi-.02,-math.pi+.02,2.8,-2.8])
def test_known_marker_behind_uses_shortest_signed_turn_up_to_pi(yaw):
    o=owner((0.,0.,yaw));o.marker_look.select(dict(focus_id=292),(0.,0.,0.))
    assert Mission.observation_turn(o)==pytest.approx(-yaw)


def test_rigid_map_frame_change_cannot_rotate_pending_odom_target():
    a=owner((0.,0.,.3));a.marker_look.select(dict(focus_id=292),(0.,0.,0.))
    b=owner((0.,0.,.3))
    # Same physical point expressed in a translated, rotated map frame.
    b.marker_focus.hints[292]['point']=np.array([-3.,2.])
    b.marker_look.select(dict(focus_id=292),(4.,3.,math.pi/2))
    b.marker_focus.hints[292]['point']=np.array([100.,100.])
    assert Mission.observation_turn(a)==pytest.approx(Mission.observation_turn(b))


def test_final_approach_id_overrides_focus_id_inherited_from_candidate():
    o=owner();o.marker_look.select(dict(approach_id=440,focus_id=292),(0.,0.,0.))
    assert Mission.observation_turn(o)==pytest.approx(math.pi/2)
    assert o.events[-1]['id']==440 and o.events[-1]['kind']=='approach'


@pytest.mark.parametrize('missing',['selection_tf','camera_tf','point'])
def test_missing_geometry_consumes_marker_task_without_generic_turn(missing):
    o=owner()
    if missing=='point':o.marker_focus.hints.clear()
    o.marker_look.select(dict(focus_id=292),None if missing=='selection_tf' else (0.,0.,0.))
    if missing=='camera_tf':
        def fail(*a):raise RuntimeError('TF unavailable')
        o.tf.lookup_transform=fail
    assert Mission.observation_turn(o)==0.
    assert o.marker_look.pending is None and o.marker_look.unavailable_count==1


def callback_owner():
    o=owner();o.marker_look.select(dict(focus_id=292),(0.,0.,0.))
    o.state='running';o.kind='nav';o.destination=(1.,0.);o.excluded=[]
    o.cancel_reason='';o.sensor_interruptions=0;o.active_goal=object();o.resume_action=None
    o.goal_handle=object();o.pending=False
    return o


def complete(o,status):
    future=Future();future.set_result(NS(status=status));Mission.completed(o,future)


def test_navigation_arrival_retains_marker_directive_for_next_observation():
    o=callback_owner();complete(o,4)
    assert o.scan_after and o.marker_look.pending is not None
    assert Mission.observation_turn(o)==0.


def test_sensor_cancel_retains_directive_and_navigation_for_resume():
    o=callback_owner();o.cancel_reason='processed_camera_stale';complete(o,5)
    assert o.marker_look.pending is not None and o.resume_action[0]=='nav'
    assert o.sensor_interruptions==1


@pytest.mark.parametrize('status,reason,contact',[(6,'',False),(5,'action_timeout',False),(5,'marker_hint',False),(5,'bump_contact',True)])
def test_failed_navigation_new_hint_and_contact_do_not_leave_stale_marker_looks(status,reason,contact):
    o=callback_owner();o.cancel_reason=reason;o.contact_pending=contact
    complete(o,status);assert o.marker_look.pending is None


def test_rejected_navigation_clears_directive():
    o=callback_owner();future=Future();future.set_result(NS(accepted=False))
    Mission.accepted(o,future)
    assert o.marker_look.pending is None and o.excluded==[(1.,0.)]


def test_nonmarker_selection_clears_old_look():
    o=callback_owner();o.marker_look.select(dict(x=3.,y=2.),(0.,0.,0.))
    assert o.marker_look.consume() is None


def test_finish_clears_pending_look_before_output_work(tmp_path):
    o=callback_owner();o.started=0.;o.goal_handle=None;o.zero=lambda:None
    o.p=lambda name:str(tmp_path);o.serializer=NS(service_is_ready=lambda:False)
    o.get_parameter=lambda name:NS(value=True);o.get_namespace=lambda:'/'
    # Run the real finish callback until its map-saving subprocess boundary.
    class StopAtSave(Exception):pass
    import unittest.mock
    with unittest.mock.patch('osp_mission.mission.subprocess.Popen',side_effect=StopAtSave):
        with pytest.raises(StopAtSave):Mission.finish(o,'timeout')
    assert o.marker_look.pending is None and o.state=='saving'
