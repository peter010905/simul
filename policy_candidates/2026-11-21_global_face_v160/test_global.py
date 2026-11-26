import math
import numpy as np
from scipy.spatial import cKDTree
from osp_mission._r32_navigation_core import Grid
from osp_mission.global_coverage import GlobalCoverage,camera_gain
from osp_mission.face_tasks import FaceTasks,face_evidence


def fit(x=2.,y=2.):
    center=np.array([x,y]);normals=np.array([[1,0],[0,1],[-1,0],[0,-1]])
    return center,[(center+.35*n,n) for n in normals],[]


def setup():
    cells=np.zeros((75,75),np.int8);cells[[0,-1],:]=100;cells[:,[0,-1]]=100
    cells[33:47,33:47]=100
    return GlobalCoverage(),Grid(cells,.05,(0.,0.,0.)),Grid(np.zeros_like(cells),.05,(0.,0.,0.))


def decide(s,g,c,elapsed=15):
    s.camera=(333.,.63,.08,18.,(0,0,0))
    return s.decide(g,c,np.array([1.,1.,0.]),(0,0,0),np.zeros((4,)+g.cells.shape,bool),elapsed,600.,lambda _:False,fitted=[fit()])


def test_blocked_side_does_not_remove_other_three_and_reopens():
    s,g,c=setup()
    c.cells[:,47:]=100
    assert decide(s,g,c)
    faces=list(s.tasks.by_key.values())
    assert len(faces)==4 and faces[0].state=='BLOCKED'
    assert any(q.get('object_face') for q in s.ranked)
    attempts=faces[0].probes
    decide(s,g,c,16)
    assert faces[0].probes==attempts
    c.cells[:,47:]=0
    decide(s,g,c,20)
    assert faces[0].state=='PENDING' and faces[0].probes==attempts+1


def test_occluder_blocks_projected_marker_and_frame_size_is_configurable():
    _,g,_=setup();point=np.array([[1.5,1.]])
    n=np.array([[-1.,0.]])
    args=(g,point,n,np.array([True]),cKDTree(point),dict(x=.55,y=1.,yaw=0.))
    assert camera_gain(*args,(333,.60,.08,18,(0,0,0)))[0]>0
    assert camera_gain(*args,(333,.60,.04,35,(0,0,0)))[0]==0
    g.cells[18:23,20]=100
    assert camera_gain(*args,(333,.60,.08,18,(0,0,0)))[0]==0


def test_one_frame_or_goal_arrival_never_marks_all_four_observed():
    s,g,c=setup();decide(s,g,c)
    f=s.tasks.by_key['1.1'];s.active_face=f.key
    s.outcome(4,'',30.)
    assert f.state=='FAILED'
    pts=np.array([[2.35,y] for y in np.linspace(1.66,2.34,18)])
    normals=np.tile(f.normal,(len(pts),1))
    face_evidence(f,f.point,f.normal,pts,normals,np.zeros(len(pts),bool),.05,.7)
    assert f.state!='OBSERVED'
    face_evidence(f,f.point,f.normal,pts,normals,np.ones(len(pts),bool),.05,.7)
    assert f.state=='OBSERVED' and len([a for a in s.tasks.by_key.values() if a.state=='OBSERVED'])==1


def test_identity_survives_reordered_fit_and_rechecks_moved_geometry():
    tasks=FaceTasks();tasks.fit([fit()],(0,0,0),0)
    c,faces,_=fit();f=tasks.by_key['1.1'];f.state='OBSERVED'
    tasks.fit([(c,list(reversed(faces)),[])],(0,0,0),10)
    assert len(tasks.boxes)==1 and tasks.by_key[f.key].state=='OBSERVED'
    tasks.fit([fit(2.2,2.)],(0,0,0),20)
    assert len(tasks.boxes)==1 and f.state=='PENDING'


def test_failed_identical_geometry_is_bounded_but_new_connectivity_retries():
    t=FaceTasks();t.fit([fit()],(0,0,0),0);f=t.by_key['1.1']
    for i in range(3):
        f.probe(b'a',100.*i);f.fail('planner_error_208',100.*i)
    assert not f.eligible(b'a',1000)
    assert not f.eligible(b'b',201)
    assert f.eligible(b'b',208)


def test_joint_visibility_rewards_both_and_unknown_does_not_count():
    _,g,_=setup();pts=np.array([[1.35,.85],[1.35,1.08]]);normals=np.array([[-1,0],[-1,0]])
    args=(g,pts,normals,np.array([True,False]),cKDTree(pts),dict(x=.55,y=1.,yaw=0.),(333,.6,.08,18,(0,0,0)))
    assert all(x>0 for x in camera_gain(*args)[:2])
    g.cells[10:30,19:22]=-1
    assert camera_gain(*args)[:2]==(0.,0.)


def test_narrow_gap_never_credits_unsafe_view_or_drops_all_object_tasks():
    s,g,c=setup()
    g.cells[:,54:56]=100
    decide(s,g,c)
    assert s.tasks.by_key['1.1'].state=='BLOCKED'
    assert any(q.get('object_face') for q in s.ranked)

def test_vertical_fov_requires_the_expected_marker_height_in_frame():
    from osp_mission.global_coverage import in_image
    optical=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    intrinsics=(300.,300.,240.,240.,480,480)
    camera=(0.,0.,0.)
    assert in_image(np.array([1.]),np.array([0.]),camera,(optical,0.,*intrinsics))[0]
    assert not in_image(np.array([1.]),np.array([0.]),camera,(optical,1.2,*intrinsics))[0]
