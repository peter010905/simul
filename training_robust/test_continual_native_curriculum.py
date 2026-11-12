import json
from pathlib import Path
import sys

import numpy as np

import continual_native_curriculum as curriculum

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/osp_mission'))
from osp_mission.marker_event_journal import MarkerEventJournal


def test_time_gate_requires_complete_safe_success_and_twenty_percent_gain():
    baseline = dict(episodes=20, success_rate=.7, contacts=0, par2_seconds=800.)
    candidate = dict(episodes=20, success_rate=.9, contacts=0, par2_seconds=600.)
    assert curriculum.materially_better(candidate, baseline, 20)
    for change in [dict(episodes=19), dict(success_rate=.85), dict(contacts=1), dict(par2_seconds=641.)]:
        assert not curriculum.materially_better(dict(candidate, **change), baseline, 20)


def test_partial_fast_failures_do_not_beat_complete_missions():
    completed = curriculum.quality([dict(success=True, seconds=500, targets=3)])
    failed = curriculum.quality([dict(success=False, seconds=10, targets=2)])
    assert failed['par2_seconds'] == 1200.
    assert curriculum.rank_key(completed) > curriculum.rank_key(failed)


def test_marker_events_keep_identity_retraction_and_approach_times(tmp_path):
    journal = MarkerEventJournal(tmp_path/'marker_events.jsonl')
    status = dict(state='running', elapsed_seconds=20., approached_ids=[])
    journal.mission(status)
    journal.detector(dict(resolved_ids=[7, 8]), status)
    journal.detector(dict(resolved_ids=[7, 8]), status)
    journal.detector(dict(resolved_ids=[7]), dict(status, elapsed_seconds=25.))
    journal.mission(dict(status, elapsed_seconds=30., approached_ids=[7]))
    rows = [json.loads(line) for line in journal.path.read_text().splitlines()]
    assert [r['id'] for r in rows if r['event'] == 'id_confirmed'] == [7, 8]
    assert next(r for r in rows if r['event'] == 'id_retracted')['id'] == 8
    assert rows[-1]['event'] == 'approach_estimated_within_1_5m'
    assert rows[-1]['elapsed_seconds'] == 30.


def test_generations_use_new_training_maps_and_paired_validation_only(tmp_path, monkeypatch):
    def fake_scene(seed, difficulty, profile):
        return dict(seed=seed, difficulty=difficulty, start=[seed, 1., 0.],
                    obstacles=[], targets=[dict(id=7)], mount_profile=profile)
    monkeypatch.setattr(curriculum, 'fresh_scene', fake_scene)
    (tmp_path/'scenarios').mkdir()
    config = dict(seed=1000, run_id='test', workspace='/candidate',
                  baseline_workspace='/baseline', baseline_weights=[1]*5)
    state = dict(jobs=[], phase='train', generation=0, mean=[0.]*5,
                 std=[.4]*5, champion=[1.]*5)
    curriculum.plan(tmp_path, config, state)
    state.update(generation=1, phase='train')
    curriculum.plan(tmp_path, config, state)
    scenes = [job['scenario']['physical_sha256'] for job in state['jobs']]
    seeds = [job['scenario']['seed'] for job in state['jobs']]
    assert len(scenes) == len(set(scenes)) == len(set(seeds)) == 40
    assert set(j['scenario']['requested_obstacles'] for j in state['jobs']) == {8, 12, 16, 24, 32}
    state['phase'] = 'pilot'
    curriculum.plan(tmp_path, config, state)
    for first, second in zip(state['jobs'][40::2], state['jobs'][41::2]):
        assert first['scenario'] == second['scenario']
        assert first['workspace'] != second['workspace']


def test_fresh_native_maps_are_accessible_and_different():
    from osp_mission.scenarios import valid
    first = curriculum.fresh_scene(1121200000, 8, 'walls')
    second = curriculum.fresh_scene(1121200101, 8, 'walls')
    assert valid(first) and valid(second)
    assert first['start'] != second['start'] or first['obstacles'] != second['obstacles']
    assert first['targets'] != second['targets']
    assert all(.08 <= t['size'] <= .30 for cfg in (first, second) for t in cfg['targets'])


def test_weight_distribution_updates_after_native_training(tmp_path):
    config = {}
    state = dict(generation=0, phase='train', planned=True, champion=[1.]*5,
                 mean=[0.]*5, std=[.4]*5, jobs=[])
    for i in range(4):
        state['jobs'].append(dict(generation=0, phase='train', role=str(i),
            parameters=[1., i+1., 1., 1., 1.],
            outcome=dict(success=i == 3, seconds=400, targets=3 if i == 3 else 1)))
    curriculum.finish_phase(tmp_path, config, state)
    assert state['champion'][1] == 4.
    assert not np.array_equal(state['mean'], [0.]*5)
    assert state['phase'] == 'pilot'
