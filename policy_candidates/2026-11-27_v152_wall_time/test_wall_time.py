""".65: a lap well along and still advancing may run past 150 s up to 210 s; white-leg cues are off."""
import ast
import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('OSP_OVERLAY', HERE / 'overlay'))   # a staged src/osp_mission also works
sys.path.insert(0, str(ROOT))

from osp_mission.wall_lap import WallLap  # noqa: E402


def lap_at(turn_share, last_credit):
    lap = WallLap(budget_fraction=.25, maximum_seconds=150.)
    lap.entry = (0., 0.)
    lap.turn = turn_share * 2 * math.pi
    lap.perimeter_m = 30.
    lap.last_credit_s = last_credit
    return lap


def test_advancing_lap_is_extended_to_210_s():
    lap = lap_at(.72, 140.)                  # the 2026-11-27 physical run: 261 deg at 150 s
    assert lap.expire(151., 600.) == 210. and lap.phase == 'perimeter' and lap.extended
    lap.last_credit_s = 190.
    assert lap.expire(209., 600.) == 210. and lap.phase == 'perimeter'
    lap.expire(210., 600.)
    assert lap.phase == 'interior' and lap.reason == 'perimeter_time_budget'


def test_stalled_or_early_lap_still_ends_at_150_s():
    stalled = lap_at(.72, 120.)              # no stop credited in the last 25 s
    stalled.expire(151., 600.)
    assert stalled.phase == 'interior'
    early = lap_at(.40, 148.)                # less than 60% of the loop
    early.expire(151., 600.)
    assert early.phase == 'interior'
    stops = lap_at(.72, 140.)                # extended, then stops stop coming
    stops.expire(160., 600.)
    assert stops.phase == 'perimeter'
    stops.expire(166., 600.)
    assert stops.phase == 'interior'


def test_short_missions_keep_their_budget():
    lap = lap_at(.9, 100.)
    lap.expire(91., 330.)                    # duration-240 = 90 s caps both budgets
    assert lap.phase == 'interior'


def test_white_leg_filter_is_off():
    source = (ROOT / 'osp_mission/phased_hybrid.py').read_text()
    tree = ast.parse(source)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'white_filter')
    body = [s for s in fn.body if not isinstance(s, ast.Expr)]
    assert len(body) == 1 and isinstance(body[0], ast.Return)
    assert isinstance(body[0].value, ast.Constant) and body[0].value.value is False


if __name__ == '__main__':
    test_advancing_lap_is_extended_to_210_s()
    test_stalled_or_early_lap_still_ends_at_150_s()
    test_short_missions_keep_their_budget()
    test_white_leg_filter_is_off()
    print('ok')
