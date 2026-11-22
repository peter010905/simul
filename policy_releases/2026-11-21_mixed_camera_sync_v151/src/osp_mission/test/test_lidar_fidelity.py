"""Finite beam geometry must rotate with the sensor when requested."""

import numpy as np
import pytest

from osp_mission.fast_sim import SearchEnv


def test_body_beams_rotate_and_invalidate_the_static_pose_cache():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[])
    env=SearchEnv(cfg,lidar_samples=640,lidar_body_frame=True)
    previous=env._lidar_cache
    original=env._scan_angles.copy()
    assert len(original)==640
    env.yaw=.31
    env.scan()
    assert env._lidar_cache is not previous
    np.testing.assert_allclose(env._scan_angles,original+.31)
    previous=env._lidar_cache
    env.scan()
    assert env._lidar_cache is previous


def test_legacy_sampling_stays_reproducible_and_lidar_does_not_cross_a_wall():
    cfg=dict(seed=1,start=[2.,2.,0.],obstacles=[],targets=[])
    env=SearchEnv(cfg)
    previous=env._lidar_cache
    env.yaw=.31;env.scan()
    assert env._lidar_cache is previous and len(env.angles)==120
    env.truth[:]=False;env.truth[:,20]=True;env.map[:]=-1
    env._lidar_pose=None;env.scan()
    assert np.any(env.map[:,20]==100)
    assert np.all(env.map[:,21:]==-1)


@pytest.mark.parametrize('count',[True,0,3.5,4096])
def test_invalid_beam_counts_are_rejected(count):
    with pytest.raises(ValueError):SearchEnv({},lidar_samples=count)
