from types import SimpleNamespace
from concurrent.futures import Future

from osp_mission.near_pass_runtime import NearPassRuntime


class Owner:
    def __init__(self):
        self.clock=100.;self.resume_action=None;self.scan_after=True;self.plan_after=0.
        self.marker_focus=SimpleNamespace(hints={42:{'stamp':90.}});self.targets={}
        self.camera_diagnostics={'accepted':100};self.sensor_interruptions=0
        self.spin_blocked_until=0.;self.recovery_pending=False;self.kind='nav'
        self.pending=False;self.cancel_reason='near_pass_inspection';self.goal_handle=None
        self.spin='spin-client';self.sent=[];self.stops=0

    def now(self):return self.clock
    def zero(self):self.stops+=1
    def send(self,client,goal,kind):self.sent.append((client,goal,kind))
    def cancel_action(self,reason):self.cancel_reason=reason


def prepared():
    owner=Owner();runtime=NearPassRuntime(owner);runtime.event=lambda *a,**kw:None
    runtime.camera_pose=lambda frame:(0.,0.,0.)
    goal=dict(turn_rad=.8,odom_camera=[0.,0.,0.],odom_yaw=.8,estimated_sim_s=3.)
    runtime.policy.reserve(goal,owner.now())
    navigation=object()
    runtime.task=dict(phase='cancel_nav',resume_goal=navigation,desired_odom_yaw=.8,
        started=owner.now(),cue_stamps={42:90.},confirmed_ids=set())
    return owner,runtime,navigation


def turned():
    owner,runtime,navigation=prepared()
    assert runtime.completed(5,'nav','near_pass_inspection')
    assert runtime.tick() and owner.sent[0][2]=='spin'
    owner.clock+=2.
    assert runtime.completed(4,'spin','')
    return owner,runtime,navigation


def test_cancel_acknowledgement_precedes_spin_and_real_frame_dwell_precedes_resume():
    owner,runtime,navigation=prepared()
    assert not owner.sent
    assert runtime.completed(5,'nav','near_pass_inspection')
    runtime.tick();assert abs(owner.sent[0][1].target_yaw-.8)<1e-9
    owner.clock+=2.;runtime.completed(4,'spin','')
    owner.clock+=.5;owner.camera_diagnostics['accepted']+=3
    runtime.tick();assert owner.resume_action is None
    owner.clock+=.4;runtime.tick()
    assert owner.resume_action==('nav',navigation) and runtime.task is None
    assert owner.targets=={} and runtime.policy.report()['spent_sim_s']>2.


def test_new_frame_for_an_existing_provisional_hint_returns_to_focus_planning():
    owner,runtime,_=turned();owner.clock+=1.;owner.camera_diagnostics['accepted']+=4
    owner.marker_focus.hints[42]['stamp']=owner.clock
    runtime.tick()
    assert owner.resume_action is None and runtime.task is None
    assert runtime.policy.history[-1]['reason']=='new_marker_evidence'


def test_sensor_cancel_preserves_original_navigation_without_restarting_spin():
    owner,runtime,navigation=turned()
    assert runtime.completed(5,'spin','processed_camera_stale')
    assert owner.resume_action==('nav',navigation) and owner.sensor_interruptions==1
    assert not runtime.tick()


def test_blocked_or_rejected_turn_uses_recovery_without_blind_navigation_resume():
    for rejected in [False,True]:
        owner,runtime,_=prepared();runtime.task['phase']='turning'
        assert runtime.rejected('spin') if rejected else runtime.completed(6,'spin','')
        assert owner.recovery_pending and owner.resume_action is None
        assert owner.spin_blocked_until==130. and not runtime.task


def test_navigation_completion_race_returns_to_the_existing_completion_handler():
    owner,runtime,_=prepared()
    assert not runtime.completed(4,'nav','near_pass_inspection')
    assert runtime.task is None and not owner.sent


def test_time_limit_marks_pending_goal_for_cancel_when_acceptance_arrives_later():
    owner,runtime,_=prepared();owner.cancel_reason='';owner.pending=True
    runtime.task['phase']='turning';owner.clock=111.
    runtime.poll()
    assert owner.cancel_reason=='near_pass_time_limit' and runtime.task is None


def test_missing_processed_frames_never_create_recognition_or_coverage_credit():
    owner,runtime,navigation=turned();owner.clock+=2.1;runtime.tick()
    assert owner.resume_action==('nav',navigation)
    assert not runtime.policy.report()['creates_recognition_or_coverage_credit']
    assert owner.camera_diagnostics['accepted']==100


def test_contact_or_mission_finish_clears_inspection_without_retaining_a_goal():
    owner,runtime,_=turned();runtime.abort('bump_contact')
    assert runtime.task is None and owner.resume_action is None and not runtime.policy.active


def test_actual_mission_callback_consumes_acknowledged_inspection_cancel_without_blacklisting():
    from osp_mission.mission import Mission
    owner,runtime,_=prepared();owner.near_pass=runtime;owner.state='running'
    result=Future();result.set_result(SimpleNamespace(status=5))
    Mission.completed(owner,result)
    assert runtime.task['phase']=='ready_turn' and not owner.sent
