import numpy as np
import pytest

from osp_mission._r32_navigation_core import Grid
from osp_mission.bearing_advance import BearingAdvance


def cue(stamp, *, size=12., x=0., yaw=0.):
    return dict(id=282, origin_odom=(x, 0.), pixels=size, yaw_odom=yaw, stamp=stamp)


def propose(memory, pose=(0., 0., 0.), time=2., blocked=False, known=(), veto=lambda goal: False):
    cells = np.zeros((72, 100), dtype=np.int8)
    if blocked:
        cells[:, 53:60] = -1
    grid = Grid(cells, .05, (-2.5, -1.8, 0.))
    safe, distances = grid.reachable(pose[:2], .32)
    return memory.choose(grid, grid, safe, safe, distances, pose, (0., 0., 0.),
                         known, time, 120., veto)


def test_need_separate_native_observations_and_known_free_ray():
    memory = BearingAdvance()
    for stamp in (1., 1., 1.2):
        memory.observe(cue(stamp), stamp)
    assert propose(memory) is None
    memory.observe(cue(1.4), 1.4)
    assert propose(memory, blocked=True) is None
    assert propose(memory, veto=lambda goal: True) is None
    assert propose(memory, known={282}) is None


def test_measured_new_evidence_for_second_advance():
    memory = BearingAdvance()
    for stamp in (1., 1.2, 1.4):
        memory.observe(cue(stamp), stamp)
    first = propose(memory)
    assert first['bearing_advance_id'] == 282
    assert 'approach_id' not in first and 'focus_id' not in first
    memory.dispatched(first)
    assert propose(memory) is None
    for t in (5., 5.2, 5.4):
        memory.observe(cue(t, x=1.2, size=12.), t)
    assert propose(memory, pose=(1.2, 0, 0), time=6.) is None
    for t in (7., 7.2, 7.4):
        memory.observe(cue(t, x=1.2, size=15.), t)
    second = propose(memory, pose=(1.2, 0, 0), time=8.)
    assert second and second['bearing_progress_m'] >= .4
    memory.dispatched(second)
    assert propose(memory, pose=(1.2, 0, 0), time=8.) is None


@pytest.mark.parametrize('bad', ['unstable', 'late', 'expired'])
def test_no_detours_for_inconsistent_or_stale_frames(bad):
    memory = BearingAdvance()
    for i,t in enumerate((1., 1.2, 1.4)):
        memory.observe(cue(t, yaw=.2*i if bad=='unstable' else 0.), t+(1.2 if bad=='late' else 0.))
    assert propose(memory, time=40. if bad=='expired' else 2.) is None
