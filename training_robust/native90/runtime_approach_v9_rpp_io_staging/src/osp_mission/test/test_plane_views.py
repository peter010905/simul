"""Physical face viewpoints must survive map translations and quarter turns."""

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.sparse.csgraph import dijkstra

from osp_mission.edge_policy import boundary_faces, options
from osp_mission.plane_views import sample,sample_all,symmetric_grid,hybrid
from osp_mission.policy import graph


def candidates(grid,robot,sampler=sample):
    safe=(grid==0)&(distance_transform_edt(grid<=50)*.1>=.32)
    row,col=int(robot[1]/.1),int(robot[0]/.1);safe[row,col]=True
    groups,_=label(safe);safe &= groups==groups[row,col]
    distances=dijkstra(graph(safe,.1),directed=False,indices=row*grid.shape[1]+col).reshape(grid.shape)
    faces=boundary_faces(grid,.1)
    return sampler(grid,.1,(0.,0.),robot,safe,distances,faces,np.zeros((4,)+grid.shape,dtype=bool))


def scene():
    grid=np.zeros((95,135),dtype=np.int8)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    grid[13:32,19:37]=100;grid[60:71,63:97]=100
    for y in range(20,65):grid[y,70+(y-20)//2]=100
    return grid


def test_fitted_face_candidates_rotate_with_observed_physical_geometry():
    grid=scene();robot=(.95,.95)
    before=set(candidates(grid,robot));assert before
    for _ in range(4):
        width=grid.shape[1]
        expected={(width-1-x,y) for y,x in before}
        robot=(robot[1],width*.1-robot[0]);grid=np.rot90(grid).copy()
        after=set(candidates(grid,robot))
        assert after==expected
        assert all(grid[y,x]==0 for y,x in after)
        before=after


def test_plane_goals_translate_with_map_origin_and_ignore_inspected_faces():
    grid=scene();state=np.zeros((4,)+grid.shape,dtype=bool)
    first,_=options(grid,.1,(0.,0.),(.95,.95),viewed=state,point_sampler=sample)
    second,_=options(grid,.1,(10.,-7.),(10.95,-6.05),viewed=state,point_sampler=sample)
    assert first and len(first)==len(second)
    for a,b in zip(first,second):
        assert a['cell']==b['cell']
        np.testing.assert_allclose([b['x']-a['x'],b['y']-a['y']],[10.,-7.],atol=1e-12)
    state[:]=True
    assert not options(grid,.1,(0.,0.),(.95,.95),viewed=state,point_sampler=sample)[0]


def test_small_cylinders_retain_viewpoints_on_all_sides():
    yy,xx=np.indices((120,120))
    for radius in [.18,.25,.4]:
        occupied=((yy+.5)*.05-3)**2+((xx+.5)*.05-3)**2<=radius**2
        grid=np.where(occupied,100,0).astype(np.int8)
        safe=distance_transform_edt(~occupied)*.05>=.32
        distances=np.hypot((yy+.5)*.05-3,(xx+.5)*.05-1.)
        faces=boundary_faces(grid,.05)
        points=sample_all(grid,.05,(0.,0.),(1.,3.),safe,distances,faces,np.zeros((4,)+grid.shape,dtype=bool))
        positions=(np.asarray(points)+.5)*.05
        assert np.any(positions[:,0]<3-radius) and np.any(positions[:,0]>3+radius)
        assert np.any(positions[:,1]<3-radius) and np.any(positions[:,1]>3+radius)
        assert all(safe[y,x] for y,x in points)


def test_short_face_sampling_also_preserves_quarter_turns():
    grid=scene();robot=(.95,.95)
    before=set(candidates(grid,robot,sample_all))
    width=grid.shape[1]
    expected={(width-1-x,y) for y,x in before}
    rotated=np.rot90(grid).copy();robot=(robot[1],width*.1-robot[0])
    assert set(candidates(rotated,robot,sample_all))==expected


def test_symmetric_spatial_coverage_and_hybrid_keep_quarter_turn_symmetry():
    for sampler in [symmetric_grid,hybrid]:
        grid=scene();robot=(.95,.95)
        before=set(candidates(grid,robot,sampler));assert before
        for _ in range(4):
            width=grid.shape[1]
            expected={(width-1-x,y) for y,x in before}
            robot=(robot[1],width*.1-robot[0]);grid=np.rot90(grid).copy()
            after=set(candidates(grid,robot,sampler))
            assert after==expected
            before=after
