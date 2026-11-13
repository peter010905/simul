"""Individually validated physical samples tolerate intermittent LiDAR fit loss."""
import pytest
from osp_mission.marker_approach import MarkerApproach


def tracker():
    value = MarkerApproach(); value.reconcile_targets({44: (1.1, 0.)}); return value


def observe(value, stamp, mode='good', point=1.1):
    distance = 1.4 if mode == 'far' else point
    evidence = dict(stamp=stamp, id=44, map_point=[distance, 0., .45],
        odom_point=[distance, 0., .45], camera_point=[0., 0., distance],
        lidar_camera_point=[0., 0., distance+.01], min_edge_pixels=30., observation_frame='map')
    if mode == 'missing': evidence['lidar_camera_point'] = None
    if mode == 'conflicting': evidence['lidar_camera_point'] = [0., 0., distance+.14]
    value.observe(evidence, (0., 0.), stamp, confirmed=True, odom_robot=(0., 0.))


def test_native_cylinder_sequence_keeps_only_three_individually_valid_frames():
    value = tracker()
    for stamp, mode in [(196.4, 'good'), (196.6, 'good'), (196.8, 'missing'),
            (197., 'conflicting'), (197.2, 'good')]: observe(value, stamp, mode)
    assert value.verified(44)
    proof = value.range_witnesses[44]
    assert [r['stamp'] for r in proof['observations']] == [196.4, 196.6, 197.2]
    assert proof['time_window_sim_s'] == 1.
    assert proof['conservative_radius_m'] == 1.3


@pytest.mark.parametrize('last_stamp, expected', [(2., True), (2.00001, False)])
def test_three_frames_need_to_fit_the_same_one_second_window(last_stamp, expected):
    value = tracker()
    for stamp in (1., 1.3, last_stamp): observe(value, stamp)
    assert value.verified(44) is expected


def test_explicit_far_frame_clears_pending_proof():
    value = tracker()
    for stamp, mode in [(1., 'good'), (1.2, 'good'), (1.4, 'far'), (1.6, 'good'), (1.8, 'good')]:
        observe(value, stamp, mode)
    assert not value.verified(44)
    observe(value, 2.)
    assert value.verified(44)


def test_inconsistent_odom_anchor_needs_three_new_consistent_frames():
    value = tracker(); observe(value, 1.); observe(value, 1.2)
    observe(value, 1.4, point=.7); observe(value, 1.6, point=.7)
    assert not value.verified(44)
    observe(value, 1.8, point=.7)
    assert value.verified(44)


def test_missing_measurements_never_create_frames_or_revoke_a_real_past_approach():
    value = tracker()
    for stamp in (1., 1.2): observe(value, stamp)
    for stamp in (1.4, 1.6, 1.8, 2., 2.2): observe(value, stamp, 'missing')
    assert not value.verified(44)
    for stamp in (3., 3.2, 3.4): observe(value, stamp)
    observe(value, 4., 'far'); observe(value, 4.2, 'missing')
    assert value.verified(44)
