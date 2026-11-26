from types import SimpleNamespace as NS
import math
import numpy as np
from osp_mission.local_commitment import CommitmentBudget,CommittedFocus,CommittedHybrid,worth_one_face
from osp_mission.marker_focus import MarkerFocus
from osp_mission.phased_hybrid import PhasedHybrid


def local(**changes):
    goal=dict(r32_kind='interior_surface',local_batch=True,previous_attempts=0,
              completed_box=False,x=.5,y=0.,yaw=0.,distance=.5,scan_angle=.4)
    goal.update(changes);return goal


def test_only_fresh_safe_local_box_face_can_precede_distant_focus():
    goal=local();focus=dict(distance=2.)
    assert worth_one_face(goal,focus,(0.,0.),0.,(4.,0.),(.5,.5),100.)
    for field,value in [('local_batch',False),('previous_attempts',1),('completed_box',True),('r32_kind','wall_surface')]:
        assert not worth_one_face(dict(goal,**{field:value}),focus,(0.,0.),0.,(4.,0.),(.5,.5),100.)
    assert not worth_one_face(goal,focus,(0.,0.),0.,(1.,0.),(.5,.5),100.)
    assert not worth_one_face(goal,focus,(0.,0.),0.,(4.,0.),(3.8,.1),100.)
    assert not worth_one_face(dict(goal,distance=5.),focus,(0.,0.),0.,(4.,0.),(.5,.5),100.)
    assert not worth_one_face(goal,focus,(0.,0.),0.,(4.,0.),(.5,.5),10.)


def test_deferral_time_and_geometry_are_invariant_under_coordinate_rotation():
    angle=1.3;c,s=math.cos(angle),math.sin(angle);rotation=np.array([[c,-s],[s,c]])
    turn=lambda p:rotation@np.array(p)+np.array([4.,-2.])
    goal=local();moved=dict(goal,x=turn((.5,0.))[0],y=turn((.5,0.))[1],yaw=angle)
    assert worth_one_face(moved,dict(distance=2.),turn((0.,0.)),angle,turn((4.,0.)),turn((.5,.5)),100.)


def test_only_dispatched_extra_face_spends_bounded_per_box_credit():
    budget=CommitmentBudget()
    assert budget.allowed((1.,2.),10.) and budget.allowed((1.,2.),11.)
    budget.dispatched((1.,2.),12.)
    assert not budget.allowed((1.1,2.),20.)
    assert budget.allowed((3.,2.),20.)
    assert budget.allowed((1.,2.),133.)


def test_unselected_focus_proposal_does_not_consume_attempt_or_cooldown(monkeypatch):
    candidate=dict(focus_id=7,x=3.,y=0.,yaw=0.,distance=3.)
    alternative=dict(local(),batch_box=(.5,.5),local_focus_commitment=True)
    hybrid=NS(local_alternative=lambda *a:alternative)
    owner=NS(hybrid=hybrid)
    focus=CommittedFocus(owner);focus.hints[7]=dict(point=np.array([4.,0.]))
    def choose(self,*args):
        self.attempts[7]=1;self.last_attempt[7]=50.;return candidate
    monkeypatch.setattr(MarkerFocus,'choose',choose)
    assert focus.choose(None,.05,(0.,0.),(0.,0.),0.,[],{},50.) is None
    assert not focus.attempts and not focus.last_attempt
    assert hybrid.focus_alternative is alternative


def test_focus_attempt_is_consumed_only_when_actually_dispatched(monkeypatch,tmp_path):
    hybrid=CommittedHybrid();hybrid.local_alternative=lambda *a:None
    owner=NS(hybrid=hybrid,now=lambda:10.,get_clock=lambda:NS(now=lambda:NS(nanoseconds=50_000_000_000)),
             p=lambda key:str(tmp_path),get_logger=lambda:NS(warning=lambda text:None))
    focus=CommittedFocus(owner);owner.marker_focus=focus;focus.hints[7]=dict(point=np.array([4.,0.]))
    def choose(self,*args):
        self.attempts[7]=1;self.last_attempt[7]=50.
        return dict(focus_id=7,x=3.,y=0.,yaw=0.,distance=3.)
    monkeypatch.setattr(MarkerFocus,'choose',choose)
    first=focus.choose(None,.05,(0.,0.),(0.,0.),0.,[],{},50.)
    assert not focus.attempts and not focus.last_attempt
    monkeypatch.setattr(PhasedHybrid,'dispatched',lambda *a:None)
    hybrid.dispatched(owner,first)
    assert focus.attempts=={7:1} and focus.last_attempt=={7:50.}


def test_confirmed_approach_wins_and_discards_unused_local_alternative():
    hybrid=CommittedHybrid();hybrid.focus_alternative=local()
    approach=dict(approach_id=103,x=1.,y=1.)
    assert hybrid.select(NS(),approach) is approach
    assert hybrid.focus_alternative is None


def test_actual_extra_face_budget_yields_via_existing_resumable_path():
    hybrid=CommittedHybrid();hybrid.active_surface=dict(goal=local(local_focus_commitment=True),started=10.)
    calls=[]
    owner=NS(now=lambda:30.6,state='running',contact_pending=False,recovery_pending=False,
        contact_inspection=NS(phase=''),goal_handle=True,pending=False,kind='nav',cancel_reason='',
        get_logger=lambda:NS(info=lambda text:None),cancel_action=calls.append,zero=lambda:calls.append('zero'))
    hybrid.poll_budget(owner)
    assert calls==['marker_hint','zero'] and hybrid.active_surface is None


def test_extra_face_budget_never_interrupts_safety_recovery():
    hybrid=CommittedHybrid();hybrid.active_surface=dict(goal=local(local_focus_commitment=True),started=10.)
    owner=NS(now=lambda:40.,state='running',contact_pending=True,recovery_pending=True,contact_inspection=NS(phase='retreat'))
    hybrid.poll_budget(owner)
    assert hybrid.active_surface is not None


def test_transit_deadline_uses_owner_elapsed_clock_not_ros_epoch():
    hybrid=CommittedHybrid();hybrid.active_surface=dict(goal=local(batch_box=(0.,0.)),started=10.)
    identity=NS(translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))
    owner=NS(now=lambda:15.,contact_pending=False,recovery_pending=False,contact_inspection=NS(phase=''),
             p=lambda key:'map' if key=='map_frame' else 'base_link',
             tf=NS(lookup_transform=lambda *args:NS(transform=identity)),destination=(.5,0.))
    assert hybrid.finish_nearby_transit(owner,dict(point=np.array([4.,0.])),1_800_000_000.)


def test_installed_mission_constructs_with_committed_components_without_starting():
    import rclpy
    from osp_mission.mission import Mission
    rclpy.init(args=['--ros-args','-p','planner:=edge','-p','use_sim_time:=true'])
    node=Mission()
    try:
        assert node.state=='idle'
        assert isinstance(node.marker_focus,CommittedFocus) and isinstance(node.hybrid,CommittedHybrid)
    finally:
        node.destroy_node();rclpy.shutdown()
