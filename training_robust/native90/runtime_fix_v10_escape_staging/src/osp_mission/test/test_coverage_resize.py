"""Map expansion may preserve evidence, but must not inspect another obstacle."""

import numpy as np
import pytest

from osp_mission.edge_policy import EdgeCoverage
from osp_mission.surface_coverage import SurfaceCoverage


@pytest.mark.parametrize('factory', [EdgeCoverage, SurfaceCoverage, lambda: SurfaceCoverage(False)])
def test_growing_and_cropping_map_preserves_physical_cell_and_partial_evidence(factory):
    data=np.zeros((40,60),dtype=np.int8);data[5:35,30]=100
    memory=factory();memory.grid(data,.1,(0.,0.))
    # Populate the physical left face; directional surface mode has bin zero.
    side=3 if isinstance(memory,EdgeCoverage) else 0
    key=(side,20,30) if memory.viewed.ndim==3 else (20,30)
    partial=(side,21,30) if memory.viewed.ndim==3 else (21,30)
    memory.viewed[key]=True;memory.counts[key]=3;memory.counts[partial]=2
    enlarged=np.pad(data,((7,9),(11,6)),constant_values=-1)
    memory.grid(enlarged,.1,(-1.1,-.7))
    shifted=(side,27,41) if memory.viewed.ndim==3 else (27,41)
    shifted_partial=(side,28,41) if memory.viewed.ndim==3 else (28,41)
    assert memory.viewed[shifted] and memory.counts[shifted_partial]==2
    assert memory.viewed.sum()==1 and not memory.viewed[shifted_partial]
    memory.grid(data,.1,(0.,0.))
    assert memory.viewed[key] and memory.counts[partial]==2


def test_unaligned_resolution_changes_and_changed_neighbours_discard_evidence():
    data=np.zeros((40,60),dtype=np.int8);data[5:35,30]=100
    for resolution,origin,change in [(.2,(0.,0.),False),(.1,(-.05,0.),False),(.1,(-1.,0.),True)]:
        memory=EdgeCoverage();memory.grid(data,.1,(0.,0.))
        memory.viewed[3,20,30]=True;memory.counts[3,20,30]=3
        new=np.pad(data,((0,0),(10,0)),constant_values=-1) if change else data.copy()
        if change:new[19,39]=100
        memory.grid(new,resolution,origin)
        assert not memory.viewed.any() and not memory.counts.any()


@pytest.mark.parametrize('factory', [EdgeCoverage, SurfaceCoverage, lambda: SurfaceCoverage(False)])
def test_native_subcell_origin_expansion_preserves_same_physical_wall(factory):
    data=np.zeros((40,60),dtype=np.int8);data[5:35,30]=100
    memory=factory();memory.grid(data,.05,(0.,0.))
    side=3 if isinstance(memory,EdgeCoverage) else 0
    key=(side,20,30) if memory.viewed.ndim==3 else (20,30)
    memory.viewed[key]=True;memory.counts[key]=3
    enlarged=np.pad(data,((9,2),(0,0)),constant_values=-1)
    # Measured native SLAM expansion: 9.211 cells rather than an integer.
    memory.grid(enlarged,.05,(0.,-.46055))
    shifted=(side,29,30) if memory.viewed.ndim==3 else (29,30)
    assert memory.viewed[shifted] and memory.counts[shifted]==3
    assert memory.viewed.sum()==1
    # A wall one metre away must never inherit that evidence.
    memory.grid(enlarged,.05,(1.,-.46055))
    assert not memory.viewed.any()
