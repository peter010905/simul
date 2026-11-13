import numpy as np
import pytest

from osp_mission.landmark_reacquisition import ObservationConsensus


def test_one_bad_seed_cannot_block_a_fresh_three_frame_cluster_forever():
    guard=ObservationConsensus();history=[]
    assert guard.update(337,history,36_400_000_000,[3.498,7.263,.41],19)=='accepted'
    for stamp in [250_000_000_000,250_200_000_000]:
        assert guard.update(337,history,stamp,[3.65,7.94,.45],40)=='inconsistent'
        assert len(history)==1
    assert guard.update(337,history,250_400_000_000,[3.66,7.94,.45],40)=='recovered'
    assert len(history)==3 and np.linalg.norm(np.median([r[1] for r in history],axis=0)-[3.65,7.94,.45])<.02
    assert len(guard.last_recovery['readable_consensus'])==3


def test_confirmed_history_never_moves_to_a_different_landmark_cluster():
    guard=ObservationConsensus();history=[(t,np.array([0.,0.,.45])) for t in [0,200_000_000,400_000_000]]
    for stamp in [1_000_000_000,1_200_000_000,1_400_000_000,1_600_000_000]:
        assert guard.update(44,history,stamp,[1.,0.,.45],40)=='inconsistent'
    assert len(history)==3 and np.allclose(history[0][1],[0,0,.45])


def test_duplicate_gap_small_or_different_cluster_cannot_recover():
    guard=ObservationConsensus();history=[(0,np.array([0.,0.,.45]))]
    for stamp in [1_000_000_000,1_000_000_000,1_000_000_000,3_000_000_000,3_200_000_000]:
        assert guard.update(44,history,stamp,[1.,0.,.45],40)!='recovered'
    assert guard.update(44,history,3_400_000_000,[1.,0.,.45],17)=='invalid'
    assert guard.update(44,history,3_600_000_000,[1.,0.,.45],float('nan'))=='invalid'
    for stamp,point in [(4_000_000_000,[1,0,.45]),(4_200_000_000,[2,0,.45]),(4_400_000_000,[1,0,.45])]:
        assert guard.update(44,history,stamp,point,40)!='recovered'
    assert len(history)==1


def test_normal_confirmation_and_spacing_remain_unchanged():
    guard=ObservationConsensus();history=[]
    assert guard.update(44,history,0,[0,0,.45],40)=='accepted'
    assert guard.update(44,history,100_000_000,[0,0,.45],40)=='duplicate'
    for stamp in [200_000_000,400_000_000]:assert guard.update(44,history,stamp,[0,0,.45],40)=='accepted'
    assert len(history)==3 and guard.last_recovery is None
    with pytest.raises(ValueError):guard.update(44,history,600_000_000,[0,0,.45],40,minimum=2)


def test_pending_consensus_is_separate_for_every_exact_id():
    guard=ObservationConsensus();histories={i:[(0,np.array([0.,0.,.45]))] for i in [337,252]}
    for ident,stamp in [(337,1_000_000_000),(252,1_100_000_000),(337,1_200_000_000)]:
        assert guard.update(ident,histories[ident],stamp,[1,0,.45],40)!='recovered'
    assert len(histories[337])==len(histories[252])==1
