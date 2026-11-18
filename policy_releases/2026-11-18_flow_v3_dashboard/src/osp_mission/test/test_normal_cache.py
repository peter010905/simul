"""Local updates must reproduce normals even at grid and thin-wall boundaries."""

import numpy as np

from osp_mission.directional_policy import normals
from osp_mission.normal_cache import AxisNormalCache


def test_incremental_normals_equal_full_recomputation_after_local_map_changes():
    rng=np.random.default_rng(59191)
    for resolution in [.05,.1,.2]:
        data=np.zeros((140,160),dtype=np.int8);data[5:130,80]=100
        memory=AxisNormalCache()
        for step in range(40):
            if step%5==0:
                y,x=[(0,0),(0,159),(139,0),(139,159)][(step//5)%4]
            else:y,x=rng.integers(0,[140,160])
            data[max(0,y-2):y+3,max(0,x-2):x+3]=rng.choice([-1,0,100])
            actual=memory.update(data,resolution);reference=normals(data,resolution)
            for a,b in zip(actual,reference):np.testing.assert_array_equal(a,b)
        assert memory.full_updates==1 and memory.local_updates>0
        before=memory.local_updates;data[data==0]=-1
        memory.update(data,resolution)
        assert memory.local_updates==before
