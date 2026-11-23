"""One readable close frame must still pass the sensor checks."""
import pytest
from osp_mission.marker_approach import MarkerApproach


def tracker():
    value = MarkerApproach(); value.reconcile_targets({44: (1.1, 0.)}); return value


def observe(value, stamp, mode='good', point=1.1):
    distance = 1.51 if mode == 'far' else point
    evidence = dict(stamp=stamp, id=44, map_point=[distance, 0., .45],
        odom_point=[distance, 0., .45], camera_point=[0., 0., distance],
        lidar_camera_point=[0., 0., distance+.01], min_edge_pixels=30., observation_frame='map')
    if mode == 'missing': evidence['lidar_camera_point'] = None
    if mode == 'conflicting': evidence['lidar_camera_point'] = [0., 0., distance+.14]
    value.observe(evidence, (0., 0.), stamp, confirmed=True, odom_robot=(0., 0.))


@pytest.mark.parametrize('distance, expected', [(1.5, True), (1.51, False)])
def test_single_frame_respects_the_distance_limit(distance, expected):
    value = tracker()
    observe(value, 1., point=distance)
    assert value.verified(44) is expected


def test_invalid_frames_cannot_grant_or_revoke_approach_proof():
    value = tracker()
    for stamp, mode in ((1., 'missing'), (1.2, 'conflicting'), (1.4, 'far')):
        observe(value, stamp, mode)
    assert not value.verified(44)

    observe(value, 1.6)
    assert value.verified(44)
    observe(value, 1.8, 'far')
    observe(value, 2., 'missing')
    assert [row['stamp'] for row in value.range_witnesses[44]['observations']] == [1.6]
