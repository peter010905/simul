"""The accelerator must preserve every beam endpoint and observed cell."""

from pathlib import Path

import numpy as np
import pytest

from osp_mission.raycast import cast


def test_native_matches_numpy_at_random_poses_and_exact_cell_boundaries():
    import osp_mission.raycast as module
    if not Path(module.__file__).with_name('_raycast.so').exists():pytest.skip('Optional native backend is not built')
    rng=np.random.default_rng(54171)
    for index in range(300):
        resolution=[.05,.1,.2][index%3]
        truth=rng.random((40,60))<.08
        pose=rng.uniform([0.,0.],[60*resolution,40*resolution])
        if index%2==0:pose=np.round(pose/resolution)*resolution
        count=[120,640][index%2]
        angles=np.linspace(-np.pi,np.pi,count,endpoint=False)+rng.uniform(-np.pi,np.pi)
        ranges=np.arange(0,12,resolution*.7)
        reference=np.full(truth.shape,-1,dtype=np.int8);actual=reference.copy()
        left=cast(truth,reference,*pose,resolution,angles,ranges,'numpy')
        right=cast(truth,actual,*pose,resolution,angles,ranges,'native')
        np.testing.assert_array_equal(actual,reference)
        for a,b in zip(left,right):np.testing.assert_array_equal(a,b)
