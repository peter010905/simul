"""The inspection tour must cover views without returning to a fixed depot."""

import numpy as np

from osp_mission.view_tour_policy import choose, options, shorten_open_tour


def test_open_tour_does_not_revisit_or_invent_a_stop():
    points=np.array([[1.,0.],[1.,3.],[4.,0.],[4.,3.]])
    start=np.linalg.norm(points,axis=1)
    pair=np.linalg.norm(points[:,None]-points[None,:],axis=2)
    order=shorten_open_tour(start,pair)
    assert sorted(order)==list(range(4))
    distance=start[order[0]]+sum(pair[a,b] for a,b in zip(order,order[1:]))
    assert distance<=10.


def test_tour_uses_reachable_uninspected_views_and_stops_when_covered():
    grid=np.zeros((45,65),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100;grid[5:40,30]=100
    viewed=np.ones((4,)+grid.shape,dtype=bool);viewed[1,5:40,30]=False
    candidates,_=options(grid,.1,(0.,0.),(4.,2.),viewed=viewed)
    goal=choose(candidates)
    assert goal is not None and grid[goal['cell']]==0 and goal['planned_view_count']>0
    viewed[:]=True
    candidates,_=options(grid,.1,(0.,0.),(4.,2.),viewed=viewed)
    assert choose(candidates) is None
