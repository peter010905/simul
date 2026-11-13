"""Behavioral guarantees for diverse training and held-out evaluation."""

import pytest

from osp_mission.curriculum import LayoutCurriculum, validate_split_separation
from osp_mission.robust_scenarios import COUNTS, PROFILES


def scenarios():
    return [dict(seed=1000 + cycle * 20 + i * 5 + j,
                 mount_profile=profile, difficulty=count,
                 obstacles=[dict(x=cycle * 20 + i * 5 + j)])
            for cycle in range(5)
            for i, profile in enumerate(PROFILES)
            for j, count in enumerate(COUNTS)]


def test_balanced_batches_exhaust_all_layouts_before_repeating():
    pool = scenarios()
    schedule = LayoutCurriculum(pool, 7)
    expected = {(profile, count) for profile in PROFILES for count in COUNTS}
    for _ in range(2):
        used = []
        for _ in range(5):
            batch = schedule.next_batch()
            assert {(c['mount_profile'], c['difficulty']) for c in batch} == expected
            used.extend(c['seed'] for c in batch)
        assert len(used) == len(set(used)) == 100
        assert set(used) == {c['seed'] for c in pool}


def test_schedule_is_reproducible_and_excludes_other_split_seeds():
    first = LayoutCurriculum(scenarios(), 17)
    second = LayoutCurriculum(list(reversed(scenarios())), 17)
    for _ in range(7):
        assert first.next_batch() == second.next_batch()
    assert first.state() == second.state()
    assert all(1000 <= seed < 1100
               for queue in first.state()['remaining_seeds'] for seed in queue)


def test_missing_strata_duplicate_seeds_and_validation_leakage_are_rejected():
    pool = scenarios()
    with pytest.raises(ValueError, match='unique'):
        LayoutCurriculum(pool + [pool[0]], 0)
    with pytest.raises(ValueError, match='Missing'):
        LayoutCurriculum([c for c in pool if c['difficulty'] != 32], 0)
    with pytest.raises(ValueError, match='seeds overlap'):
        validate_split_separation(pool, [pool[0]])
    with pytest.raises(ValueError, match='geometry overlaps'):
        validate_split_separation(pool, [dict(pool[0], seed=9999)])
    validate_split_separation(pool, [dict(pool[0], seed=9999,
                                        obstacles=[dict(x=9999)])])


def test_resume_preserves_exact_future_layouts_across_shuffle_boundary():
    original=LayoutCurriculum(scenarios(),41)
    for _ in range(3):original.next_batch()
    recovered=LayoutCurriculum(scenarios(),999)
    recovered.restore(original.state())
    for _ in range(9):
        assert original.next_batch()==recovered.next_batch()
        assert original.state()==recovered.state()
