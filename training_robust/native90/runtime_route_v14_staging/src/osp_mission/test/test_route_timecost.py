import math
import numpy as np
from osp_mission.edge_policy import choose


def candidate(gain,scan,distance=1.):
    return dict(features=np.array([gain,0.,distance,0.,0.]),scan_angle=scan)


def test_equal_information_prefers_short_observation_action():
    full=candidate(5.,2*math.pi);partial=candidate(5.,math.pi/3)
    assert choose([full,partial]) is partial


def test_high_information_full_scan_is_still_allowed():
    full=candidate(20.,2*math.pi);partial=candidate(1.,math.pi/3)
    assert choose([partial,full]) is full


def test_closer_goal_does_not_win_if_it_requires_disproportionate_scan_time():
    near=candidate(5.,2*math.pi,.5);far=candidate(5.,math.pi/3,1.)
    assert choose([near,far]) is far
    assert choose([]) is None
