from types import SimpleNamespace
import numpy as np
from osp_mission.interior_priority import InteriorPriority, hinted
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import CenterWall


def grid():
    return Grid(np.zeros((100,100),np.int8),.1,(0.,0.,0.))


def view(x,y,**kw):
    return dict(x=x,y=y,yaw=0.,distance=1.,features=[0.,1.,1.,0.,0.],**kw)


def choose(interiors=(),frontiers=(),walls=(),remaining=600.,dist=None):
    g=grid(); distances=np.ones((100,100)) if dist is None else dist
    return InteriorPriority().choose(g,distances,(0.,0.,0.),interiors,frontiers,walls,[],remaining,lambda _:False)


def test_nearby_fresh_box_is_serviced_before_an_ordinary_wall_or_frontier():
    box=view(4.,4.,batch_box=[4.5,4.5],previous_attempts=0)
    frontier=view(8.,8.)
    wall=view(4.,1.,surface_map=[4.,0.])
    assert choose([(2.,box)],[frontier],[(1.,wall)])['batch_box']==[4.5,4.5]


def test_new_floor_favours_interior_over_wall_when_costs_are_equal():
    wall=view(5.,1.,surface_map=[5.,0.])
    result=choose(frontiers=[view(5.,.5),view(5.,3.)],walls=[(1.,wall)])
    assert result['y']==3. and result['known_wall_distance_m']==3.


def test_missing_camera_credit_still_offers_interior_surface():
    surface=view(4.,4.,previous_attempts=0)
    assert choose([(2.,surface)])['interior_priority']


def test_unreachable_finished_and_time_infeasible_work_cannot_force_a_goal():
    box=view(4.,4.,completed_box=True)
    assert choose([(1.,box)]) is None
    assert choose(frontiers=[view(5.,5.)],dist=np.full((100,100),np.inf)) is None
    assert choose([(100.,view(4.,4.))],remaining=10.) is None


def test_wall_id_or_candidate_keeps_explicit_priority():
    assert hinted(dict(weak_wall=True))
    assert hinted(dict(bearing_advance_id=42))
    assert hinted(dict(focus_id=42))
    assert not hinted(dict(r32_kind='wall_surface'))


def test_real_selector_honours_a_wall_marker_before_a_local_box(monkeypatch):
    s=CenterWall();g=grid();s.survey.camera=(350.,1.2)
    marker=view(2.,2.,weak_wall=True)
    monkeypatch.setattr(s.weak,'choose',lambda *args,**kw:marker)
    monkeypatch.setattr(s.continuity,'choose',lambda *a: (_ for _ in ()).throw(AssertionError('marker should win')))
    result=s.decide(g,g,(1.,1.,0.),(0.,0.,0.),np.zeros((4,100,100),bool),
                    10.,600.,lambda _:False)
    assert result is marker and s.reason=='wall_marker_evidence'
