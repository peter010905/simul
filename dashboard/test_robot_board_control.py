"""HTTP commands use fake controls/processes; these tests cannot move a robot."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_board import make_handler
from robot_board_control import BoardControl, valid_request


def headers(token='secret', host='127.0.0.1:8767', origin='http://127.0.0.1:8767'):
    return {'Host': host, 'Origin': origin, 'Content-Type': 'application/json', 'X-OSP-Control-Token': token}


def test_only_same_origin_board_json_with_token_can_command():
    assert valid_request(headers(), 'secret')
    assert not valid_request(headers(token='wrong'), 'secret')
    assert not valid_request(headers(origin='https://elsewhere.example'), 'secret')
    assert not valid_request(headers(host='rebind.example', origin='http://rebind.example'), 'secret')
    assert not valid_request({'Host': '127.0.0.1:8767'}, 'secret')


@pytest.fixture
def board_server():
    control = SimpleNamespace(token='secret', request=Mock(return_value={'phase': 'queued'}))
    monitor = SimpleNamespace(snapshot=lambda: {'read_only': False}, lock=threading.RLock())
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(monitor, control))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = 'http://127.0.0.1:' + str(server.server_port)
    yield url, control
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


def test_get_and_cross_origin_requests_never_start(board_server):
    url, control = board_server
    with urlopen(url + '/api/state?start=true') as result:
        assert result.status == 200
    for request in (Request(url + '/api/control/start'),
                    Request(url + '/api/control/start', data=b'{}', headers={'Origin': 'https://elsewhere.example'})):
        with pytest.raises(HTTPError):
            urlopen(request)
    control.request.assert_not_called()


@pytest.mark.parametrize('action', ['start', 'start-v3-mppi', 'start-mixed', 'start-upstream-r32', 'stop'])
def test_authorized_post_dispatches_exactly_one_action(board_server, action):
    url, control = board_server
    request = Request(url + '/api/control/' + action, data=b'{}', headers={
        'Origin': url, 'Content-Type': 'application/json', 'X-OSP-Control-Token': 'secret'})
    with urlopen(request) as result:
        assert result.status == 202
        assert json.load(result)['phase'] == 'queued'
    control.request.assert_called_once_with(action)


def test_wrong_body_and_unknown_action_never_dispatch(board_server):
    url, control = board_server
    for path, body in (('/api/control/start', b'{"command":"anything"}'), ('/api/control/reboot', b'{}')):
        request = Request(url + path, data=body, headers={
            'Origin': url, 'Content-Type': 'application/json', 'X-OSP-Control-Token': 'secret'})
        with pytest.raises(HTTPError):
            urlopen(request)
    control.request.assert_not_called()


def process(pid, code=None):
    return SimpleNamespace(pid=pid, returncode=code, poll=lambda: code)


def test_double_start_rejected_and_stop_cancels_even_queued_wrapper(tmp_path):
    control = BoardControl(tmp_path, tmp_path, 184, state_dir=tmp_path)
    with patch.object(control, 'spawn', side_effect=[(process(111), tmp_path/'start.log'),
                                                   (process(222), tmp_path/'stop.log')]) as spawn, \
         patch('robot_board_control.os.killpg') as kill:
        assert control.snapshot()['phase'] == 'idle'
        spawn.assert_not_called()
        assert control.request('start')['phase'] == 'queued'
        with pytest.raises(RuntimeError):
            control.request('start')
        assert control.request('stop')['phase'] == 'stopping'
        kill.assert_called_once_with(111, 15)
        control.request('stop')
        assert spawn.call_count == 2


def test_missing_sensor_failure_remains_visible_and_retryable(tmp_path):
    control = BoardControl(tmp_path, tmp_path, 184, state_dir=tmp_path)
    control.start_process = process(111, 1)
    control.start_log = tmp_path/'start.log'
    control.start_log.write_text('TimeoutError: Preflight: waiting_for_scan\n')
    (tmp_path/'session.json').write_text(json.dumps({'phase': 'not_started', 'requested_wall_epoch_s': 1.}))
    result = control.snapshot()
    assert result['phase'] == 'failed' and result['can_start'] and result['can_stop']
    assert 'LiDAR' in result['detail']


def test_start_command_uses_same_prebuilt_session_and_recorder(tmp_path):
    control = BoardControl(tmp_path, tmp_path/'runtime', 184, state_dir=tmp_path)
    command = control.command('start')
    assert str(tmp_path/'run_policy.sh') in command
    assert str(tmp_path/'scripts/robot_session.py') in command
    assert 'start' in command and '--workspace' in command
    assert 'colcon' not in command
    stop = control.command('stop')
    assert 'stop' in stop and str(tmp_path/'run_policy.sh') not in stop


@pytest.mark.parametrize(('action','workspace'), [('start-r32','r32_drive_original18_ws'),
                                                ('start-v3-mppi','face_intent_v3_sweep_ws'),
                                                ('start-mixed','flow_v3_r32_print_ws')])
def test_alternate_start_uses_its_own_overlay_and_shares_the_original_stop_lock(tmp_path, action, workspace):
    board = BoardControl(tmp_path, tmp_path/'baseline', 184, state_dir=tmp_path)
    cmd = board.command(action)
    assert cmd[cmd.index('--workspace')+1] == str(tmp_path/workspace)
    assert cmd[cmd.index('--')+1:].count('start') == 1
    assert '2026-11-13_face_intent_v2/policy.json' in cmd[cmd.index('--policy')+1]
    with patch.object(board, 'spawn', return_value=(process(1), tmp_path/'start.log')):
        board.request(action)
        with pytest.raises(RuntimeError):
            board.request('start')


def test_upstream_separate_detector_common_stop_and_interlock(tmp_path):
    board = BoardControl(tmp_path, tmp_path/'default', 184, state_dir=tmp_path)
    cmd = board.command('start-upstream-r32')
    assert cmd[cmd.index('--workspace') + 1] == str(tmp_path/'r32_upstream_b1ed300_ws')
    assert cmd[cmd.index('--runtime') + 1] == 'upstream-r32'
    assert '--policy' not in cmd and 'run_policy.sh' not in ' '.join(cmd)
    with patch.object(board, 'spawn', return_value=(process(123), tmp_path/'cmd.log')):
        assert board.request('start-upstream-r32')['policy'] == '원본 · R32'
        for action in ('start-mixed', 'start-v3-mppi'):
            with pytest.raises(RuntimeError):
                board.request(action)
        with patch('robot_board_control.os.killpg'):
            assert board.request('stop')['phase'] == 'stopping'
