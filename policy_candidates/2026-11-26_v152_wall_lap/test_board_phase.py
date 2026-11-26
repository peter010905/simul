from robot_board_core import mission_display


def test_stage_labels_follow_actual_mission_state():
    assert mission_display(dict(state='running',wall_lap=dict(phase='perimeter')))['label']=='빠른 벽 순회 중'
    assert mission_display(dict(state='running',wall_lap=dict(phase='interior')))['label']=='가운데 박스 탐색 중'


def test_finished_mission_does_not_keep_an_active_stage_label():
    text=mission_display(dict(state='done',task_success=False,termination_reason='timeout',
                             wall_lap=dict(phase='interior')))
    assert text['label']=='시간 초과'
    assert mission_display(dict(state='running'))['label']=='탐색 · 주행 중'
