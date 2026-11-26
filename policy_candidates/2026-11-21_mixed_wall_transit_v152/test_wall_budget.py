"""Control-flow regression using a mocked feasible interior proposal, not Nav2 proof."""
import numpy as np

from osp_mission import _r32_navigation_core as r32
from osp_mission.center_wall import CenterWall
from osp_mission.phased_hybrid import PhasedHybrid


def decide(monkeypatch, elapsed, probe_only=False, walls=True):
    data = np.zeros((100, 100), dtype=np.int8)
    if walls:
        data[70, :] = 100
        data[71:, :] = -1
    grid = r32.Grid(data, .1, (0., 0., 0.))
    cost = r32.Grid(np.zeros_like(data), .1, (0., 0., 0.))

    def interior(*args, **kwargs):
        fields = args[9]
        return 1., dict(fields, x=6., y=5., yaw=0., distance=1.)

    monkeypatch.setattr('osp_mission.center_wall.inspection_pose', interior)
    schedule = CenterWall()
    fitted = [(np.array([6., 5.]), [(np.array([6., 5.]), np.array([-1., 0.]))], [])]
    goal = schedule.decide(grid, cost, np.array([5., 5., 0.]), (0., 0., 0.),
        np.zeros((4,)+data.shape, dtype=bool), elapsed, 600., lambda goal: False,
        fitted=fitted, probe_only=probe_only)
    return schedule, goal


def test_late_fresh_interior_no_longer_bypasses_real_wall_candidates(monkeypatch):
    schedule, goal = decide(monkeypatch, 550.)
    assert schedule.counts['interior'] > 0 and schedule.counts['wall'] > 0
    assert goal['r32_kind'] == 'wall_surface'
    assert schedule.reason == 'remaining_time_for_mapped_walls'


def test_interior_stays_first_early_or_when_no_wall_candidates_exist(monkeypatch):
    assert decide(monkeypatch, 100.)[1]['r32_kind'] == 'interior_surface'
    assert decide(monkeypatch, 550., walls=False)[1]['r32_kind'] == 'interior_surface'


def test_probe_contract_cannot_turn_an_interrupt_query_into_wall_navigation(monkeypatch):
    assert decide(monkeypatch, 550., probe_only=True)[1]['r32_kind'] == 'interior_surface'


def test_registered_marker_approach_still_bypasses_surface_scheduler():
    goal = dict(approach_id=7)
    hybrid = PhasedHybrid.__new__(PhasedHybrid)
    assert hybrid.select(None, goal, None, None, None, None) is goal
