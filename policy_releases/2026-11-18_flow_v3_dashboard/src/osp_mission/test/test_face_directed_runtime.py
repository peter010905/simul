import math
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest

from osp_mission.face_directed_look import FaceDirectedLook
from osp_mission.mission import Mission
from test_marker_directed_runtime import owner,callback_owner


def prepared(yaw=.2):
    o=owner((0.,0.,yaw));o.face_look=FaceDirectedLook(o);o.face_look.event=lambda *a,**kw:None
    o.face_look.select(dict(face_inspection=True,yaw=.2,scan_angle=math.pi/3),(0.,0.,0.))
    return o


def test_selected_sixty_degree_face_look_cannot_be_replaced_with_negative_generic_sweep(monkeypatch):
    o=prepared();monkeypatch.setattr('osp_mission.mission.select_observation_sweep',lambda *a:pytest.fail('intent overwritten'))
    assert Mission.observation_turn(o)==pytest.approx(math.pi/3)
    assert o.face_look.consume() is None


def test_arrival_heading_error_and_map_frame_rotation_are_accounted_for():
    o=prepared(.3)
    assert Mission.observation_turn(o)==pytest.approx(math.pi/3-.1)
    o=prepared(.3);o.face_look.select(dict(face_inspection=True,yaw=.2-.5,scan_angle=math.pi/3),(2.,3.,.5))
    assert Mission.observation_turn(o)==pytest.approx(math.pi/3-.1)


@pytest.mark.parametrize('marker_key',['focus_id','approach_id'])
def test_marker_override_inheriting_face_metadata_does_not_create_stale_face_sweep(marker_key):
    o=prepared();o.face_look.select(dict(face_inspection=True,yaw=.2,scan_angle=math.pi/3,**{marker_key:292}),(0.,0.,0.))
    assert o.face_look.consume() is None


def test_explicit_marker_look_precedes_face_sweep():
    o=prepared();o.marker_look.select(dict(focus_id=292),(0.,0.,0.))
    assert Mission.observation_turn(o)==pytest.approx(-.2)


def test_missing_transform_consumes_face_task_without_generic_turn():
    o=prepared();o.face_look.select(dict(face_inspection=True,yaw=.2,scan_angle=math.pi/3),None)
    assert Mission.observation_turn(o)==0. and o.face_look.unavailable==1


@pytest.mark.parametrize('status,reason',[(4,''),(5,'processed_camera_stale'),(6,''),(5,'marker_hint')])
def test_real_navigation_callback_preserves_only_success_or_sensor_resume(status,reason):
    o=callback_owner();o.face_look=prepared().face_look;o.cancel_reason=reason
    future=Future();future.set_result(NS(status=status));Mission.completed(o,future)
    assert (o.face_look.pending is not None)==(status==4 or reason=='processed_camera_stale')


def test_larger_arrival_error_never_exceeds_existing_one_hundred_twenty_degree_limit():
    o=prepared(-.8);assert abs(Mission.observation_turn(o))<=2*math.pi/3
    assert not o.face_look.report()['creates_coverage_or_recognition_credit']
