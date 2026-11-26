"""Run in the staged ROS workspace alongside its navigation_flow test fixture."""
from types import SimpleNamespace
from test_navigation_flow import owner
from osp_mission.mission import Mission


def mission():
    robot, events, _ = owner(active=True, protected=True)
    robot.contact_pending = False
    robot.approach = SimpleNamespace(report=lambda: {'verified_ids': []}, verified=lambda ident: False,
                                     complete=lambda targets: False, reconcile_targets=lambda targets: None)
    robot.get_logger = lambda: SimpleNamespace(info=lambda *a: None, debug=lambda *a: None)
    return robot, events


def register(robot, ident):
    point = SimpleNamespace(position=SimpleNamespace(x=1., y=0.))
    Mission.on_targets(robot, SimpleNamespace(markers=[SimpleNamespace(id=ident, ns='point', pose=point)]))


def test_new_id_interrupts_even_during_face_transit_and_duplicate_does_not():
    robot, events = mission()
    register(robot, 15)
    Mission.tick(robot)
    assert events == ['registered_marker']
    register(robot, 15)
    Mission.tick(robot)
    assert events == ['registered_marker']


def test_during_recovery_registration_is_retained_until_safe():
    robot, events = mission()
    robot.contact_pending = True
    register(robot, 8)
    Mission.tick(robot)
    assert not events and robot.registered_pending == {8}
    robot.contact_pending = False
    Mission.tick(robot)
    assert events == ['registered_marker']


def test_current_marker_approach_can_continue():
    robot, events = mission()
    robot.batch_approach_id = 8
    register(robot, 8)
    Mission.tick(robot)
    assert not events
