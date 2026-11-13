from concurrent.futures import Future
from types import SimpleNamespace
from osp_mission.mission import Mission
from nav2_msgs.action import NavigateToPose,Spin
import pytest
import math


def test_failed_spin_enters_cooldown_and_retreat_instead_of_retrying():
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='spin',scan_after=True,
                          recovery_pending=False,spin_blocked_until=0.,now=lambda:100.,
                          get_logger=lambda:SimpleNamespace(warning=lambda message:None))
    result=Future(); result.set_result(SimpleNamespace(status=6))
    Mission.completed(owner,result)
    assert owner.goal_handle is None
    assert owner.spin_blocked_until==130.
    assert not owner.scan_after
    assert owner.recovery_pending


def test_retreat_failure_returns_to_planning_not_another_retreat():
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='backup',scan_after=True,now=lambda:100.)
    result=Future(); result.set_result(SimpleNamespace(status=6))
    Mission.completed(owner,result)
    assert owner.goal_handle is None
    assert not owner.scan_after
    assert owner.plan_after==101.


def scan_owner():
    sent=[]
    owner=SimpleNamespace(state='running',targets={},collision_count=0,started=1.,
                          status=SimpleNamespace(publish=lambda message:None),now=lambda:100.,
                          p=lambda key:600.,ready=lambda:True,goal_handle=None,pending=False,
                          plan_after=0.,recovery_pending=False,scan_after=True,spin_blocked_until=0.,
                          surface_mode=True,initial_scan=False,next_scan_angle=math.pi/3,
                          spin=object(),zero=lambda:None,
                          send=lambda client,goal,kind:sent.append((goal,kind)))
    owner.edge_mode=False;owner.inspect=False
    owner.observation_turn=lambda:Mission.observation_turn(owner)
    owner.get_logger=lambda:SimpleNamespace(info=lambda message:None)
    return owner,sent


def test_successful_scan_does_not_suppress_the_next_nearby_view():
    owner,sent=scan_owner()
    Mission.tick(owner)
    assert len(sent)==1 and owner.spin_blocked_until==0.
    owner.kind='spin'
    result=Future();result.set_result(SimpleNamespace(status=4))
    Mission.completed(owner,result)
    assert owner.plan_after==101.5
    # Arrival at another view three seconds later must still allow a scan.
    owner.now=lambda:103.;owner.kind='nav';owner.destination=(1.,2.);owner.excluded=[]
    Mission.completed(owner,result)
    Mission.tick(owner)
    assert len(sent)==2 and owner.spin_blocked_until==0.


def test_still_camera_view_dwells_without_sending_a_zero_angle_action():
    owner,sent=scan_owner();owner.next_scan_angle=0.
    Mission.tick(owner)
    assert not sent and not owner.scan_after
    assert owner.plan_after==101.5 and owner.spin_blocked_until==0.


@pytest.mark.parametrize('angle,progress,remaining',[(6.28,1.8,4.48),(-3.,-1.,-2.),(1.,1.1,0.)])
def test_sensor_canceled_spin_retains_remaining_angle_without_obstacle_recovery(angle,progress,remaining):
    goal=Spin.Goal();goal.target_yaw=angle
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='spin',
        cancel_reason='processed_camera_stale',active_goal=goal,spin_progress=progress,
        sensor_interruptions=0,spin_blocked_until=0.,recovery_pending=False,
        get_logger=lambda:SimpleNamespace(warning=lambda s:None))
    future=Future();future.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,future)
    assert owner.resume_action[0]=='spin'
    assert owner.resume_action[1].target_yaw==pytest.approx(remaining)
    assert owner.spin_blocked_until==0. and not owner.recovery_pending
    assert owner.sensor_interruptions==1


def test_sensor_canceled_navigation_does_not_blacklist_a_valid_view():
    goal=NavigateToPose.Goal();goal.pose.pose.position.x=2.5
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='nav',
        cancel_reason='scan_stale',active_goal=goal,sensor_interruptions=0,
        excluded=[],destination=(2.5,3.),get_logger=lambda:SimpleNamespace(warning=lambda s:None))
    future=Future();future.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,future)
    assert owner.excluded==[]
    assert owner.resume_action[0]=='nav' and owner.resume_action[1].pose.pose.position.x==2.5


def test_old_spin_feedback_cannot_change_new_action_progress():
    owner=SimpleNamespace(action_generation=2,kind='spin',spin_progress=0.)
    message=SimpleNamespace(feedback=SimpleNamespace(angular_distance_traveled=2.))
    Mission.spin_feedback(owner,message,1)
    assert owner.spin_progress==0.
    Mission.spin_feedback(owner,message,2)
    assert owner.spin_progress==2.


@pytest.mark.parametrize('kind',['nav','spin','backup'])
def test_contact_cancels_action_and_does_not_resume_or_scan(kind):
    canceled=[];zeros=[]
    owner=SimpleNamespace(state='running',kind=kind,now=lambda:100.,last_bump=0.,
        collision_count=0,goal_handle=object(),pending=False,scan_after=True,
        resume_action=object(),cancel_action=canceled.append,zero=lambda:zeros.append(True),
        excluded=[],destination=(1.,2.))
    hazard=SimpleNamespace(detections=[SimpleNamespace(type=1)])
    Mission.on_hazard(owner,hazard)
    assert canceled==['bump_contact'] and zeros
    assert owner.collision_count==1 and owner.resume_action is None
    future=Future();future.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,future)
    assert not owner.scan_after and owner.resume_action is None
    assert owner.recovery_pending==(kind!='backup')
    assert owner.spin_blocked_until==130. and owner.plan_after==100.5


@pytest.mark.parametrize('status,scan',[(4,True),(5,False),(6,False)])
def test_only_successful_arrivals_trigger_observation(status,scan):
    owner=SimpleNamespace(goal_handle=object(),state='running',kind='nav',
        excluded=[],destination=(1.,2.),scan_after=True)
    future=Future();future.set_result(SimpleNamespace(status=status))
    Mission.completed(owner,future)
    assert owner.scan_after is scan
    assert owner.excluded==[(1.,2.)]
