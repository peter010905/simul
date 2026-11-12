"""Boundary tests for final evidence, stale runs, media access, and seeking."""
import importlib.util
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

spec = importlib.util.spec_from_file_location('robot_observatory', Path(__file__).with_name('server.py'))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))


def result():
    return {'reason': 'all_targets_found', 'elapsed_seconds': 200, 'targets': 3, 'collisions': 0,
            'ground_truth_audit': {'available': True, 'correct_ids': True, 'correct_positions': True,
                                  'all_approached': True, 'approached_within_1_5m': 3, 'approach_radius_m': 1.5,
                                  'found_ids': [1, 2, 3], 'closest_approach_m': {'1': 1.2}}}


def test_success_requires_independent_current_rubric():
    value = result()
    assert server.evaluate(value)['passed']
    for field, replacement in [('available', False), ('correct_ids', False),
                               ('correct_positions', False), ('approached_within_1_5m', 2),
                               ('approach_radius_m', 2.0)]:
        broken = result()
        broken['ground_truth_audit'][field] = replacement
        assert not server.evaluate(broken)['passed']
    value['collisions'] = 1
    assert not server.evaluate(value)['passed']
    value = result()
    value['elapsed_seconds'] = 600.1
    assert not server.evaluate(value)['passed']
    assert not server.evaluate({'success': True, 'targets': 3})['passed']


@pytest.fixture
def store(tmp_path):
    output = tmp_path / 'training_robust/native90/campaign/jobs/0'
    episode = output / 'episodes/123'
    save(output.parents[1] / 'state.json', {'status': 'running', 'jobs': [
        {'output': str(output), 'number': 0, 'status': 'running', 'domain': 114,
         'unit': 'osp-test.service', 'scenario': {'seed': 123, 'actual_obstacles': 8}}]})
    save(episode / 'live_status.json', {'status': {'state': 'running', 'targets': 2,
         'approached_ids': [1], 'elapsed_seconds': 190}, 'detector': {'resolved_ids': [1, 2]}})
    observatory = server.Observatory(tmp_path, tmp_path / 'cache', cameras=False)
    observatory.services = lambda names: {'osp-test.service': {'ActiveState': 'inactive'}}
    observatory.refresh()
    return observatory, output, episode


def test_completed_evaluation_overrides_stale_live_and_stopped_service(store):
    obs, output, episode = store
    identifier = server.key(output)
    assert obs.detail(identifier)['state'] == 'stopped'
    save(episode / 'evaluation.json', result())
    obs.refresh()
    detail = obs.detail(identifier)
    assert detail['state'] == 'passed'
    assert detail['targets'] == 3
    assert detail['confirmed_ids'] == [1, 2, 3]
    assert detail['audit']['actual_approached'] == 3
    assert detail['camera']['live'] is False


def test_unfinalized_recording_is_not_a_playable_video(store):
    obs, _, episode = store
    (episode / 'video').mkdir()
    (episode / 'video/mission.mp4').write_bytes(b'0123456789')
    save(episode / 'evaluation.json', result())
    obs.refresh()
    assert not obs.snapshot['videos']
    save(episode / 'video/recording.json', {'frames': 5})
    obs.refresh()
    assert len(obs.snapshot['videos']) == 1


def test_failed_exact_credit_audit_overrides_original_pass_in_job_and_video(store):
    obs, output, episode = store
    save(episode/'evaluation.json', result())
    save(output/'observer_analysis.json', {'physical_credit_audit': {
        'all_three_physical_credits_verified': False}})
    save(episode/'video/recording.json', {'frames': 5})
    (episode/'video/mission.mp4').write_bytes(b'0123456789')
    obs.refresh()
    detail = obs.detail(server.key(output))
    assert detail['state'] == 'failed'
    assert detail['audit']['original_benchmark_passed']
    assert detail['audit']['physical_credit_verified'] is False
    assert not detail['audit']['passed']
    assert not obs.snapshot['videos'][0]['passed']


def test_successful_credit_audit_is_visible_without_rewriting_original_result(store):
    obs, output, episode = store
    save(episode/'evaluation.json', result())
    save(output/'observer_analysis.json', {'physical_credit_audit': {
        'all_three_physical_credits_verified': True}})
    obs.refresh()
    detail = obs.detail(server.key(output))
    assert detail['state'] == 'passed'
    assert detail['audit']['physical_credit_verified'] is True
    assert json.loads((episode/'evaluation.json').read_text()) == result()


def test_invalid_environment_is_never_a_valid_policy_success(store):
    obs, output, episode = store
    save(episode / 'evaluation.json',result())
    save(output / 'environment_invalidation.json',dict(policy_training_sample_valid=False,reason='Invalid cliff input'))
    save(episode / 'video/recording.json',dict(frames=5))
    (episode / 'video/mission.mp4').write_bytes(b'0123456789')
    obs.refresh()
    detail=obs.detail(server.key(output))
    assert detail['state']=='invalid_environment'
    assert not detail['audit']['passed']
    assert detail['audit']['raw_rubric_passed']
    assert obs.snapshot['videos'][0]['environment_valid'] is False
    assert not obs.snapshot['videos'][0]['passed']


@pytest.mark.parametrize('value,expected', [('bytes=0-2', (0, 2)), ('bytes=7-', (7, 9)),
                                           ('bytes=-4', (6, 9)), ('bytes=0-999', (0, 9))])
def test_ranges(value, expected):
    assert server.byte_range(value, 10) == expected


@pytest.mark.parametrize('value', ['bytes=10-', 'bytes=-0', 'bytes=2-1', 'bytes=0-1,3-4', 'bytes=-'])
def test_bad_ranges(value):
    with pytest.raises(ValueError):
        server.byte_range(value, 10)


def test_http_registry_blocks_path_traversal_and_supports_seek(store):
    obs, output, episode = store
    save(episode / 'evaluation.json', result())
    save(episode / 'video/recording.json', {'frames': 5})
    (episode / 'video/mission.mp4').write_bytes(b'0123456789')
    obs.refresh()
    http = server.ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    http.observatory = obs
    thread = threading.Thread(target=http.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{http.server_port}'
    try:
        identifier = obs.snapshot['videos'][0]['id']
        with urlopen(Request(base + '/media/video?id=' + identifier, headers={'Range': 'bytes=3-6'})) as response:
            assert response.status == 206
            assert response.headers['Content-Range'] == 'bytes 3-6/10'
            assert response.read() == b'3456'
        with urlopen(Request(base + '/media/video?id=' + identifier, method='HEAD')) as response:
            assert response.headers['Content-Length'] == '10'
            assert response.read() == b''
        for path in ['/../../etc/passwd', '/api/artifact?id=' + server.key(output) + '&name=../../etc/passwd', '/media/video?id=../../etc/passwd']:
            with pytest.raises(HTTPError) as failure:
                urlopen(base + path)
            assert failure.value.code == 404
        with pytest.raises(HTTPError) as failure:
            urlopen(Request(base + '/media/video?id=' + identifier, headers={'Range': 'bytes=12-'}))
        assert failure.value.code == 416
        assert failure.value.headers['Content-Range'] == 'bytes */10'
        with urlopen(base + '/api/job?id=' + server.key(output)) as response:
            assert json.load(response)['audit']['passed']
    finally:
        http.shutdown()
        http.server_close()


def test_tail_is_bounded_and_ignores_partial_first_line(tmp_path):
    path = tmp_path / 'launch.log'
    path.write_text('secret partial' + 'x' * 100 + '\nline 1\nline 2\n')
    assert server.tail(path, limit=2, max_bytes=30) == 'line 1\nline 2'
