import math
import numpy as np
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall
from osp_mission.object_survey import ObjectSurvey
from osp_mission.face_tasks import FaceTasks,face_evidence

def fit():
    center=np.array([2.,2.]);normals=np.array([[1,0],[0,1],[-1,0],[0,-1]])
    return center,[(center+.35*n,n) for n in normals],[]

def setup():
    cells=np.zeros((90,110),np.int8);cells[[0,-1],:]=100;cells[:,[0,-1]]=100;cells[33:47,33:47]=100
    grid=Grid(cells,.05,(0,0,0));cost=Grid(np.zeros_like(cells),.05,(0,0,0))
    scheduler=CenterWall();scheduler.survey=ObjectSurvey()
    scheduler.survey.perimeter.camera=(333.,math.radians(71.))
    return scheduler,grid,cost,np.zeros((4,)+cells.shape,bool)

def choose(s,g,c,seen,t=1,choices=()):
    return s.decide(g,c,(1.,1.,0.),(0.,0.,0.),seen,t,600.,lambda _:False,choices=choices,fitted=[fit()])

def test_one_blocked_face_keeps_three_and_reopens_on_cost_change():
    s,g,c,seen=setup();c.cells[:,47:]=100
    result=choose(s,g,c,seen)
    assert 'object_face' in result
    f=s.survey.tasks.by_key['1.1'];assert f.state=='BLOCKED'
    count=f.probes
    choose(s,g,c,seen,50);assert f.probes==count
    c.cells[:,47:]=0
    choose(s,g,c,seen,60)
    assert f.state=='PENDING' and f.probes==count+1

def test_four_observed_faces_retire_all_owned_generic_targets_and_leave():
    s,g,c,seen=setup();choose(s,g,c,seen)
    for f in s.survey.tasks.by_key.values():f.state='OBSERVED';f.fraction=1.
    # No frontier-gain requirement. The existing candidate must still be safe.
    goal=dict(x=4.2,y=3.2,yaw=0.,scan_angle=0.,features=np.zeros(5),distance=4.)
    result=choose(s,g,c,seen,40,[goal])
    assert (result.get('region_transfer') or result.get('perimeter_section')) and s.survey.finished=={1}
    choose(s,g,c,seen,80,[goal])
    assert len(s.survey.tasks.boxes)==1 and s.survey.finished=={1}
    assert all(f.state=='OBSERVED' for f in s.survey.tasks.by_key.values())

def test_camera_evidence_is_required_and_latched_until_geometry_changes():
    tasks=FaceTasks();tasks.fit([fit()],(0,0,0),1)
    f=tasks.by_key['1.1']
    xy=np.array([[2.35,y] for y in np.linspace(1.66,2.34,20)])
    normals=np.tile(f.normal,(20,1))
    face_evidence(f,f.point,f.normal,xy,normals,np.zeros(20,bool),.05,.70)
    assert f.state=='PENDING'
    face_evidence(f,f.point,f.normal,xy,normals,np.ones(20,bool),.05,.70)
    assert f.state=='OBSERVED'
    face_evidence(f,f.point,f.normal,np.empty((0,2)),np.empty((0,2)),np.zeros(0,bool),.05,.70)
    assert f.state=='OBSERVED'
    center,faces,seen=fit();tasks.fit([(center+.2,[(p+.2,n) for p,n in faces],seen)],(0,0,0),20)
    assert f.state=='PENDING'

def test_region_commitment_survives_marker_cancel_but_failed_path_is_cooled():
    m=ObjectSurvey();m.region=np.array([4.,3.])
    m.outcome(5,'marker_hint',100)
    assert m.region is not None
    m.outcome(6,'',110)
    assert m.region is None and len(m.rejections)==1

def test_nav_arrival_without_post_sweep_does_not_finish_face():
    s,g,c,seen=setup();goal=choose(s,g,c,seen)
    s.survey.dispatched(goal,1);key=goal['object_face']
    s.survey.outcome(4,'',10)
    assert s.survey.tasks.by_key[key].state!='OBSERVED'
    choose(s,g,c,seen,15)
    assert s.survey.tasks.by_key[key].state=='FAILED'

def test_completed_object_cannot_return_through_generic_fallback():
    s,g,c,seen=setup();choose(s,g,c,seen)
    for f in s.survey.tasks.by_key.values():f.state='OBSERVED';f.fraction=1.
    seen[:]=True
    result=choose(s,g,c,seen,40,[])
    assert result is None or s.survey.owner(result,(0.,0.,0.)) is None
    assert s.survey.finished=={1}

def test_real_region_dispatch_can_be_replanned_with_numpy_coordinates():
    s,g,c,seen=setup()
    s.survey.observe_pose((1.,1.,0.),(0.,0.,0.))
    goal=dict(x=4.2,y=3.2,yaw=0.,scan_angle=0.,features=np.zeros(5),distance=4.)
    _,distance=g.reachable((1.,1.),.32)
    def select(t):
        return s.survey.next_region(g,c,(1.,1.,0.),(0.,0.,0.),[goal],distance,np.ones_like(c.cells,bool),lambda _:False,t,600.)
    first=select(40);s.survey.dispatched(first,40)
    assert s.survey.region is not None
    assert select(50)['region_transfer']
    s.survey.observe_pose((4.2,3.2,0.),(0.,0.,0.))
    assert s.survey.region is None
