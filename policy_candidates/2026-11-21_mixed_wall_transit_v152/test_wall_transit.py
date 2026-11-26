import math
from types import SimpleNamespace

import numpy as np

from osp_mission.wall_transit_inspection import WallTransitInspection
from osp_mission.wall_transit_runtime import WallTransitRuntime


def opportunity(policy=None, **changes):
    data = np.zeros((100, 100), dtype=np.int8)
    data[70, :] = 100
    data[71:, :] = -1
    viewed = np.zeros((4,)+data.shape, dtype=bool)
    args = dict(data=data, resolution=.1, origin=(0., 0.), camera=(5., 5., 0.),
        robot=(5., 5.), viewed=viewed, focal=332., fov=1.25,
        odom_camera=(1., 2., 0.), now=100., elapsed=100., remaining=500.)
    args.update(changes)
    return (policy or WallTransitInspection()).choose(**args)


def test_side_wall_outside_the_old_1_4m_radius_gets_a_bounded_look():
    goal = opportunity()
    assert goal and goal['range_min_m'] > 1.4
    assert .6 <= goal['gain_surface_m'] and abs(goal['turn_rad']) <= math.radians(100)
    assert goal['range_max_m'] <= 3.5 and goal['estimated_sim_s'] < 10


def test_box_island_is_left_to_the_existing_box_scheduler():
    data = np.zeros((100, 100), dtype=np.int8)
    data[65:75, 45:55] = 100
    assert opportunity(data=data) is None


def test_seen_wall_and_wall_already_in_front_do_not_interrupt():
    viewed = np.ones((4, 100, 100), dtype=bool)
    assert opportunity(viewed=viewed) is None
    viewed[:, 70, 45:55] = False
    original = viewed.copy()
    assert opportunity(viewed=viewed, camera=(5., 5., math.pi/2)) is None
    assert np.array_equal(viewed, original)


def test_unknown_or_too_close_footprint_and_occluded_wall_are_rejected():
    for value in (-1, 100):
        data = np.zeros((100, 100), dtype=np.int8)
        data[70, :] = 100
        data[71:, :] = -1
        data[50, 50] = value
        assert opportunity(data=data) is None
    data[50, 50] = 0
    data[50, 52] = 100
    assert opportunity(data=data) is None
    data[50, 52] = 0
    data[60, :] = -1
    assert opportunity(data=data) is None


def test_protected_marker_navigation_cooldown_and_budget_are_preserved():
    assert opportunity(protected=True) is None
    assert opportunity(elapsed=10) is None
    assert opportunity(remaining=40) is None
    policy = WallTransitInspection()
    policy.spent = 35
    assert opportunity(policy) is None
    policy = WallTransitInspection()
    goal = opportunity(policy)
    policy.reserve(goal, 100)
    assert opportunity(policy, now=110) is None
    policy.finish(104, 'processed_frame_dwell')
    assert policy.spent >= goal['estimated_sim_s']
    assert opportunity(policy, now=110) is None
    assert policy.report()['spent_sim_s'] == 4
    assert not policy.report()['creates_recognition_or_coverage_credit']


def test_same_physical_strip_is_not_retried_after_map_translation():
    viewed = np.ones((4, 100, 100), dtype=bool)
    viewed[:, 70, 47:54] = False
    policy = WallTransitInspection()
    goal = opportunity(policy, viewed=viewed)
    assert goal is not None
    policy.reserve(goal, 100)
    policy.finish(104, 'processed_frame_dwell')
    assert opportunity(policy, viewed=viewed, origin=(2., 3.), camera=(7., 8., 0.),
                       robot=(7., 8.), now=200, elapsed=200) is None


class Owner:
    def __init__(self):
        self.clock = 100.
        self.resume_action = None
        self.scan_after = True
        self.plan_after = 0.
        self.marker_focus = SimpleNamespace(hints={})
        self.targets = {}
        self.camera_diagnostics = {'accepted': 100}
        self.sensor_interruptions = 0
        self.spin_blocked_until = 0.
        self.recovery_pending = False
        self.kind = 'nav'
        self.pending = False
        self.cancel_reason = ''
        self.goal_handle = None
        self.spin = 'spin-client'
        self.sent = []
        self.finished = None
        self.state = 'running'

    def now(self): return self.clock
    def zero(self): pass
    def cancel_action(self, reason):
        if not self.cancel_reason:
            self.cancel_reason = reason
    def send(self, client, goal, kind): self.sent.append((client, goal, kind))
    def finish(self, reason): self.finished = reason


def prepared():
    owner = Owner()
    runtime = WallTransitRuntime(owner)
    runtime.event = lambda *a, **kw: None
    runtime.camera_pose = lambda frame: (0., 0., 0.)
    goal = opportunity()
    runtime.policy.reserve(goal, 100.)
    navigation = object()
    runtime.task = dict(phase='cancel_nav', resume_goal=navigation, desired_odom_yaw=.8,
        started=100., cue_stamps={}, confirmed_ids=set())
    return owner, runtime, navigation


def test_timeout_waits_for_pending_spin_cancel_before_resuming_original_goal():
    owner, runtime, goal = prepared()
    runtime.task['phase'] = 'turning'
    owner.pending = True
    owner.clock = 111.
    runtime.poll()
    assert runtime.task and runtime.task['timed_out'] and owner.resume_action is None
    runtime.tick()
    assert not owner.sent and owner.cancel_reason == 'near_pass_time_limit'
    owner.pending = False
    assert runtime.completed(5, 'spin', 'near_pass_time_limit')
    assert runtime.task is None and owner.resume_action == ('nav', goal)
    assert not owner.recovery_pending


def test_new_marker_before_turn_replans_without_spinning_or_resuming_old_goal():
    owner, runtime, _ = prepared()
    runtime.task['phase'] = 'ready_turn'
    owner.targets[7] = (1., 2.)
    assert runtime.tick()
    assert not owner.sent and runtime.task is None and owner.resume_action is None


def test_new_marker_during_spin_cancels_then_replans_without_spin_failure_recovery():
    owner, runtime, _ = prepared()
    runtime.task['phase'] = 'turning'
    owner.goal_handle = object()
    owner.targets[7] = (1., 2.)
    assert runtime.tick() and runtime.task is not None
    assert owner.cancel_reason == 'registered_marker' and owner.resume_action is None
    owner.goal_handle = None
    assert runtime.completed(5, 'spin', 'registered_marker')
    assert runtime.task is None and not owner.recovery_pending and owner.resume_action is None


def test_real_blocked_spin_still_uses_original_recovery_and_no_blind_resume():
    owner, runtime, _ = prepared()
    runtime.task['phase'] = 'turning'
    assert runtime.completed(6, 'spin', '')
    assert owner.recovery_pending and owner.resume_action is None and runtime.task is None


def test_sensor_loss_is_recorded_even_when_a_marker_arrives_during_the_look():
    owner, runtime, goal = prepared()
    owner.targets[7] = (1., 2.)
    assert runtime.completed(5, 'spin', 'processed_camera_stale')
    assert owner.sensor_interruptions == 1 and owner.resume_action == ('nav', goal)
    assert runtime.task is None


def test_lost_action_result_never_resumes_over_an_unknown_spin():
    owner, runtime, _ = prepared()
    runtime.task['phase'] = 'turning'
    runtime.poll()
    assert runtime.task is None and owner.resume_action is None
    assert owner.finished == 'wall_transit_action_state_unknown'


def test_actual_mission_callback_gives_a_new_marker_priority_over_the_active_look():
    from concurrent.futures import Future
    from osp_mission.mission import Mission
    owner, runtime, _ = prepared()
    runtime.task['phase'] = 'turning'
    owner.near_pass = runtime
    owner.kind = 'spin'
    owner.cancel_reason = 'registered_marker'
    owner.targets[7] = (1., 2.)
    result = Future()
    result.set_result(SimpleNamespace(status=5))
    Mission.completed(owner, result)
    assert runtime.task is None and owner.resume_action is None and not owner.recovery_pending


def test_frame_audit_distinguishes_a_real_dwell_from_missing_processed_frames():
    for count in (0, 4):
        owner, runtime, goal = prepared()
        runtime.task.update(phase='dwell', frames_before=100, dwell_started=100.)
        owner.clock = 102.1
        owner.camera_diagnostics['accepted'] += count
        events = []
        runtime.event = lambda event, **details: events.append((event, details))
        assert runtime.tick() and runtime.task is None
        observed = next(d for e, d in events if e == 'wall_view_frames')
        assert observed['minimum_met'] is (count >= 3) and observed['processed_frames'] == count
        assert owner.resume_action == ('nav', goal) and not owner.targets


def test_generic_face_reservation_does_not_disable_wall_looks_on_the_transit(monkeypatch):
    from osp_mission.near_pass_runtime import NearPassRuntime
    owner = Owner()
    owner.face_look = SimpleNamespace(pending={'end_odom_yaw': 1.})
    runtime = WallTransitRuntime(owner)
    calls = []
    monkeypatch.setattr(NearPassRuntime, 'request', lambda self: calls.append(True))
    runtime.request()
    assert calls == [True] and owner.face_look.pending == {'end_odom_yaw': 1.}


def test_actual_marker_reservation_still_prevents_optional_wall_looks(monkeypatch):
    from osp_mission.near_pass_runtime import NearPassRuntime
    owner = Owner()
    owner.marker_look = SimpleNamespace(pending={'kind': 'approach'})
    runtime = WallTransitRuntime(owner)
    calls = []
    monkeypatch.setattr(NearPassRuntime, 'request', lambda self: calls.append(True))
    runtime.request()
    assert not calls
