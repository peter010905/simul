import numpy as np
import pytest
from osp_mission.landmark_identity import resolved_ids
from osp_mission.marker_approach import MarkerApproach


def observe(approach, distance, source='depth', provisional=False):
    approach.observe(dict(stamp=2., id=751, map_point=[0., 0.], observation_frame='map',
                          close_base_xy=[distance, 0.], range_source=source, provisional=provisional),
                     [0., 0.], 2.1)


@pytest.mark.parametrize('distance,source,provisional,expected', [
    (1.5, 'lidar', False, True), (1.49, 'depth', False, True),
    (1.5001, 'depth', False, False), (float('nan'), 'depth', False, False),
    (.7, 'multiview', False, False), (.8, 'depth', True, False)])
def test_one_exposure(distance, source, provisional, expected):
    policy = MarkerApproach(1)
    observe(policy, distance, source, provisional)
    targets = {751: (0., 0.)}
    assert not policy.complete(targets)
    policy.reconcile_targets(targets)
    assert policy.complete(targets) == expected
    policy.reconcile_targets({})
    assert not policy.verified(751)


def test_only_ranged_ids_bypass_repeat_reads_and_conflicts_still_apply():
    histories = {751: [(2., np.zeros(3))], 15: [(2., np.zeros(3))]}
    assert resolved_ids(histories, close_read_ids={751})[0] == {751}
    assert not resolved_ids(histories, close_read_ids={751, 15})[0]
