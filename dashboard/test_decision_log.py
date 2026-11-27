"""Decision timeline: incremental tailing, Korean lines, merged repeats, session time base."""
import json
import os
import tempfile
import time
import unittest
from pathlib import Path

from decision_log import DecisionLog, artifacts_dir


def write(path, rows, end='\n'):
    with path.open('a') as stream:
        for row in rows:
            stream.write(json.dumps(row) + end)


class DecisionLogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / 'run-1' / 'artifacts'
        self.root.mkdir(parents=True)

    def tearDown(self):
        self.tmp.cleanup()

    def test_timeline_from_policy_files(self):
        lap = dict(phase='perimeter', reason='following_measured_wall', path_m=3.0, confirmed_stops=2)
        write(self.root / 'center_wall.jsonl', [
            dict(elapsed=9.0, phase='wall', reason='quick_perimeter_pass', wall_lap=lap),
            dict(elapsed=20.0, phase='wall', reason='quick_perimeter_pass', wall_lap=lap),
            dict(elapsed=152.6, phase='interior', reason='white_leg_candidate',
                 wall_lap=dict(lap, phase='interior', reason='perimeter_time_budget', path_m=24.7, confirmed_stops=11)),
        ])
        write(self.root / 'startup.jsonl', [dict(event='entry', elapsed_s=0.), dict(event='prepared', elapsed_s=10.7),
                                            dict(event='mission_started', elapsed_s=13.),
                                            dict(event='bumper_recovery', elapsed_s=621.8, contacts=3)])
        write(self.root / 'box_continuity.jsonl', [dict(event='face_failed', elapsed=165.3, reason='no_progress_10s')])
        write(self.root / 'marker_directed_looks.jsonl', [dict(event='cleared', stamp=234.5, reason='registered_marker', id=903)])
        log = DecisionLog()
        log.follow(self.root)
        log.poll()
        entries = log.snapshot()['entries']
        titles = [(e['t'], e['title']) for e in reversed(entries)]
        self.assertEqual(titles[1:3], [(0.0, '미션 시작'), (9.0, '벽 순회')])
        lap_entry = next(e for e in entries if e['title'] == '벽 순회')
        self.assertEqual(lap_entry.get('repeat'), 2)                       # 9 s and 20 s merged
        self.assertIn((152.6, '벽 순회 시간이 끝나 내부 탐색으로 전환'), titles)
        self.assertIn((152.6, '흰 다리 후보'), titles)
        self.assertIn((165.3, '상자 면 관찰 실패'), titles)
        self.assertIn((234.5, '마커 등록'), titles)
        self.assertIn((608.8, '범퍼 접촉 3회'), titles)
        self.assertEqual(titles[0], (-2.3, '주행 준비 완료'))                 # before the mission start                    # session time minus mission start
        self.assertEqual(log.snapshot()['run'], 'run-1')

    def test_partial_lines_wait_for_the_rest(self):
        path = self.root / 'center_wall.jsonl'
        path.write_text('{"elapsed": 5.0, "phase": "interior", "reason": "finish_cur')
        log = DecisionLog()
        log.follow(self.root)
        log.poll()
        self.assertEqual(log.snapshot()['entries'], [])
        with path.open('a') as stream:
            stream.write('rent_box"}\n')
        log.poll()
        self.assertEqual([e['title'] for e in log.snapshot()['entries']], ['상자'])

    def test_switching_runs_resets(self):
        write(self.root / 'center_wall.jsonl', [dict(elapsed=1., phase='wall', reason='quick_perimeter_pass')])
        log = DecisionLog()
        log.follow(self.root)
        log.poll()
        other = Path(self.tmp.name) / 'run-2' / 'artifacts'
        other.mkdir(parents=True)
        log.follow(other)
        log.poll()
        self.assertEqual(log.snapshot(), dict(run='run-2', entries=[]))

    def test_artifacts_dir_prefers_session_inside_run_logs(self):
        logs = Path(self.tmp.name)
        newer = logs / 'run-3' / 'artifacts'
        newer.mkdir(parents=True)
        os.utime(newer, (time.time() + 10, time.time() + 10))
        self.assertEqual(artifacts_dir({'output': str(self.root)}, logs), self.root.resolve())
        self.assertEqual(artifacts_dir({'output': '/etc'}, logs), newer.resolve())
        self.assertEqual(artifacts_dir({}, logs), newer.resolve())
        self.assertIsNone(artifacts_dir({}, logs / 'missing'))


if __name__ == '__main__':
    unittest.main()
