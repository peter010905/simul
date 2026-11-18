from osp_mission.startup_state import StartupTransitions


def test_configure_then_activate_after_observing_each_real_state():
    policy=StartupTransitions()
    assert [policy.next(state) for state in [1,10,2,13,3]]==[1,None,3,None,None]


def test_already_active_slam_never_receives_a_reset_or_reactivation():
    policy=StartupTransitions()
    assert all(policy.next(3) is None for _ in range(10))
    assert policy.attempts=={1:0,3:0}


def test_lost_configuration_reply_is_reconciled_from_actual_state():
    policy=StartupTransitions()
    assert policy.next(1)==1
    # The server configured despite the lost response. Re-query says inactive.
    assert policy.next(2)==3
    assert policy.next(3) is None
    assert policy.attempts=={1:1,3:1}


def test_pending_transition_and_unknown_states_cannot_trigger_duplicates():
    policy=StartupTransitions()
    for state in [0,1,2,10,13]:assert policy.next(state,pending=True) is None
    assert policy.next(0) is None and policy.attempts=={1:0,3:0}


def test_failed_transition_retries_are_bounded_but_late_success_is_respected():
    policy=StartupTransitions()
    for _ in range(3):assert policy.next(1)==1
    assert policy.next(1) is None and policy.exhausted(1)
    assert policy.next(2)==3 and not policy.exhausted(2)
