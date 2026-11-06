"""A nominal rotation model must enforce its declared motion limits."""

import math
import numpy as np
import pytest

from osp_mission.angular_motion import RotationProfile
from osp_mission.fast_sim import SearchEnv
from osp_mission.policy import wrap


def test_short_turn_is_triangular_and_long_turn_reaches_speed_limit():
    short=RotationProfile(.04,1.,2.)
    assert short.cruise==0.
    assert short.duration==pytest.approx(2*math.sqrt(.04/2.))
    long=RotationProfile(2*math.pi,1.,3.2)
    assert long.duration==pytest.approx(2*math.pi+.3125)
    for profile in [short,long]:
        times=np.linspace(0,profile.duration,10001)
        positions=np.array([profile.position(t) for t in times])
        speeds=np.diff(positions)/np.diff(times)
        assert (speeds>=-1e-10).all() and speeds.max()<=profile.peak_speed+1e-9
        assert profile.position(0)==pytest.approx(0.)
        assert profile.position(profile.duration)==pytest.approx(profile.angle)
        acceleration=np.diff(speeds)/(times[1]-times[0])
        assert np.abs(acceleration).max()<=profile.acceleration+1e-6


def test_nominal_spin_respects_camera_cadence_and_deadline():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[])
    env=SearchEnv(cfg,seed=1,camera_hz=5.,angular_profile='trapezoid',max_turn_speed=1.,max_turn_acceleration=3.2)
    frames=[];env.detect=lambda:frames.append(env.elapsed)
    env.rotate(-2*math.pi,deadline=1.)
    assert env.elapsed==pytest.approx(1.)
    np.testing.assert_allclose(frames,[.2,.4,.6,.8,1.],atol=1e-9)
    assert env.yaw==pytest.approx(wrap(-RotationProfile(2*math.pi,1.,3.2).position(1.)))


def test_invalid_angular_limits_are_rejected():
    with pytest.raises(ValueError):RotationProfile(1.,float('nan'),3.2)
    with pytest.raises(ValueError):RotationProfile(1.,1.,0.)
