"""Range/map quantization must not erase observations or inspect distant faces."""
import numpy as np
from osp_mission.edge_policy import EdgeCoverage
from osp_mission.surface_hits import match


def test_subcell_hit_can_confirm_near_side_but_not_the_opposite_side():
    data=np.zeros((80,100),dtype=np.int8);data[10:70,60]=100
    memory=EdgeCoverage()
    # A range just before the wall falls into the neighbouring free cell.
    for stamp in [0.,.2,.4]:
        memory.observe(data,.05,(0.,0.),(1.025,2.025,0.),[[2.995,2.025]],stamp,665.,1.25)
    assert memory.viewed[3,40,60]
    assert not memory.viewed[1,40,60]


def test_endpoints_cannot_match_unrelated_distant_walls_or_wrap_grid_edges():
    data=np.zeros((80,100),dtype=np.int8);data[:,60]=100;data[:,-1]=100
    points,ys,xs=match(data,.05,(0.,0.),[[2.8,2.025],[-.01,2.025],[float('nan'),2.]])
    assert len(points)==len(ys)==len(xs)==0
