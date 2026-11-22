"""Physical approach semantics; map drift must neither grant nor erase proof."""
import numpy as np
import pytest

from osp_mission.marker_approach import MarkerApproach
from osp_mission.native_audit import NativeAudit


def frame(stamp, ident=44, distance=1.2):
    return dict(stamp=stamp, id=ident, map_point=[distance, 0., .45],
        odom_point=[distance, 0., .45], camera_point=[0., 0., distance],
        lidar_camera_point=[0., 0., distance+.01],
        min_edge_pixels=30., observation_frame='map')


def observe(tracker, stamp, ident=44, distance=1.2):
    tracker.observe(frame(stamp, ident, distance), (0., 0.), stamp,
        confirmed=True, odom_robot=(0., 0.))


def test_map_only_proximity_never_grants_completion_even_at_zero_distance():
    tracker = MarkerApproach(); targets = {i: (0., 0.) for i in (1, 2, 3)}
    tracker.reconcile_targets(targets)
    tracker.observe_pose(targets, (0., 0.), 1., 1.)
    assert len(tracker.positions) == 3
    assert not tracker.complete(targets)
    assert tracker.report()['verified_ids'] == []


def test_all_three_ids_need_three_physical_frames_and_keep_proof_after_map_shift():
    tracker = MarkerApproach(); targets = {i: (1.2, 0.) for i in (1, 2, 3)}
    tracker.reconcile_targets(targets)
    for ident in targets:
        for stamp in (1., 1.2): observe(tracker, stamp, ident)
    assert not tracker.complete(targets)
    for ident in targets: observe(tracker, 1.4, ident)
    assert tracker.complete(targets)
    corrected = {i: (20., -5.) for i in targets}
    tracker.reconcile_targets(corrected)
    assert tracker.complete(corrected)
    assert not tracker.complete({1: (0., 0.), 2: (0., 0.)})
    assert tracker.report()['required_actual_radius_m'] == 1.5
    assert tracker.report()['sensor_proof_radius_m'] == 1.3


@pytest.mark.parametrize('mode', ['unconfirmed', 'provisional', 'small', 'range_conflict',
    'outside_buffer', 'missing_odom', 'missing_depth', 'null_odom', 'null_lidar', 'nan', 'stale', 'wrong_frame'])
def test_unreadable_unconfirmed_stale_or_inconsistent_evidence_cannot_complete(mode):
    tracker = MarkerApproach(); tracker.reconcile_targets({44: (1.2, 0.)})
    for stamp in (1., 1.2, 1.4, 1.6):
        evidence = frame(stamp); base = (0., 0.); now = stamp
        if mode == 'provisional': evidence['provisional'] = True
        if mode == 'small': evidence['min_edge_pixels'] = 17.
        if mode == 'range_conflict': evidence['lidar_camera_point'] = [0., 0., 1.4]
        if mode == 'outside_buffer': evidence['odom_point'] = [1.31, 0., .45]
        if mode == 'missing_odom': base = None
        if mode == 'missing_depth': evidence.pop('camera_point')
        if mode == 'null_odom': evidence['odom_point'] = None
        if mode == 'null_lidar': evidence['lidar_camera_point'] = None
        if mode == 'nan': evidence['odom_point'][0] = float('nan')
        if mode == 'stale': now += 1.
        if mode == 'wrong_frame': evidence['observation_frame'] = 'odom'
        tracker.observe(evidence, (0., 0.), now, confirmed=mode != 'unconfirmed', odom_robot=base)
    assert not tracker.verified(44)


def test_duplicates_and_old_frames_do_not_count_but_missing_samples_preserve_recent_valid_frames():
    tracker = MarkerApproach(); tracker.reconcile_targets({44: (1.2, 0.)})
    for stamp in (1., 1., 1., 3., 3.2): observe(tracker, stamp)
    assert not tracker.verified(44)
    bad = frame(3.4); bad['min_edge_pixels'] = 9.
    tracker.observe(bad, (0., 0.), 3.4, confirmed=True, odom_robot=(0., 0.))
    observe(tracker, 3.6)
    assert tracker.verified(44)
    assert [r['stamp'] for r in tracker.range_witnesses[44]['observations']] == [3., 3.2, 3.6]


def test_retracting_identity_erases_proof_and_prevents_a_stale_revisit_exemption():
    tracker = MarkerApproach(); tracker.reconcile_targets({44: (1.2, 0.)})
    for stamp in (1., 1.2, 1.4): observe(tracker, stamp)
    assert tracker.verified(44)
    tracker.reconcile_targets({})
    tracker.reconcile_targets({44: (10., 10.)})
    assert not tracker.verified(44)
    assert not any(44 in m for m in (tracker.hints, tracker.range_witnesses, tracker.witness_runs))


def test_approach_goal_reserves_stopping_margin_and_respects_a_blocked_shortcut():
    tracker = MarkerApproach()
    tracker.observe(dict(id=1, stamp=1., map_point=[3., 3.], observation_frame='map'), [0., 3.], 1.)
    grid = np.zeros((80, 80), dtype=int)
    result = tracker.choose(grid, .1, (0., 0.), (1., 3.), 0., [], {1: (3., 3.)}, 10.)
    assert result is not None
    assert .65 <= np.linalg.norm(np.array([result['x'], result['y']])-[3., 3.]) <= .9+1e-9
    grid[:, 15:18] = 100; tracker.last_attempt.clear()
    assert tracker.choose(grid, .1, (0., 0.), (1., 3.), 0., [], {1: (3., 3.)}, 30.) is None


@pytest.mark.parametrize('distance,expected', [(1.49, True), (1.5, True), (1.51, False), (2., False)])
def test_independent_evaluator_keeps_actual_one_point_five_limit(tmp_path, distance, expected):
    audit = NativeAudit({'targets': [{'id': 1, 'x': 3., 'y': 3.}]})
    audit.calibrate((0., 0., 0.), (0., 0., 0.)); audit.observe(1., 3.-distance, 3., 0.)
    path = tmp_path/'markers.csv'; path.write_text('id,x,y\n1,3,3\n')
    result = audit.evaluate(path)
    assert result['all_approached'] == expected
    assert result['correct_ids'] and result['correct_positions']
