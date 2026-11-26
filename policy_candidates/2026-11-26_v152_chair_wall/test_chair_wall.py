"""Checks for the .51 chair-wall candidate (no ROS node, no robot)."""
import math

import numpy as np
import pytest

from osp_mission.chair_band import ChairBand, ChairBandTracker
from osp_mission.wall_lap import WallLap
from osp_mission._r32_navigation_core import Grid

RES = 0.05


def room(chairs=True, length=20.0, width=11.0, west_depth=1.4, east_depth=2.6, seed=3, jagged=0.3):
    rng = np.random.default_rng(seed)
    h, w = round(width / RES) + 20, round(length / RES) + 20
    cells = np.full((h, w), 0, np.int8)
    cells[:10] = cells[-10:] = -1
    cells[:, :10] = cells[:, -10:] = -1
    # solid long walls only; glass ends are not walls
    cells[10:14, 10:-10] = 100
    cells[-14:-10, 10:-10] = 100
    def leg(x, y):
        r, c = int(y / RES), int(x / RES)
        cells[r, c] = 100
    # table columns: 4 legs each, three rows
    for cx in np.arange(4.0, length - 4.0, 2.6):
        for cy in (width / 2 - 2.7, width / 2, width / 2 + 2.7):
            for dx in (-.4, .4):
                for dy in (-.9, .9):
                    leg(cx + dx + .5, cy + dy + .5)
    truth = {}
    if chairs:
        for side, depth in (("west", west_depth), ("east", east_depth)):
            inner = []
            for y in np.arange(1.2, width - .4, .55):
                d = depth + rng.uniform(-jagged, jagged)
                rows = np.arange(.3, d, .55)
                inner.append(rows[-1] + .19)  # innermost leg of this row
                for k in rows:
                    x = k if side == "west" else length - k
                    for dx in (-.19, .19):
                        for dy in (-.19, .19):
                            leg(x + dx + .5, y + dy + .5)
            truth[side] = float(np.max(inner))
    return cells, truth


def test_detects_both_glass_ends_and_not_tables():
    cells, truth = room()
    band = ChairBand(cells, RES, (0., 0., 0.))
    assert len(band.ends) == 2
    assert abs(band.axis) < math.radians(3)  # u runs along x
    for e in band.ends:
        # the virtual wall sits just inside the innermost chair leg of a jagged edge
        depth = e["wall_u"] - .5 if e["sign"] < 0 else 20.5 - e["wall_u"]
        side = "west" if e["sign"] < 0 else "east"
        assert -.15 < depth - truth[side] < .45


def test_no_wall_without_chairs():
    cells, _ = room(chairs=False)
    assert ChairBand(cells, RES, (0., 0., 0.)).ends == []


def test_unfinished_short_map_needs_a_strong_band():
    cells, _ = room(length=9.0, chairs=False)
    tracker = ChairBandTracker()
    for _ in range(3):
        tracker.update(cells, RES, (0., 0., 0.))
    assert tracker.walls == {}


def test_commit_needs_two_consecutive_updates():
    cells, _ = room()
    tracker = ChairBandTracker()
    tracker.update(cells, RES, (0., 0., 0.))
    assert tracker.walls == {}
    tracker.update(cells, RES, (0., 0., 0.))
    assert len(tracker.walls) == 2
    # inside the room is not excluded; inside a band is
    assert not tracker.contains((10.5, 6.0))
    assert tracker.contains((0.8, 6.0))


def test_lap_budget_is_four_minutes():
    from osp_mission.center_wall import CenterWall
    lap = CenterWall().wall_lap
    assert lap.expire(0., 600.) == pytest.approx(240.)


def test_chair_face_uses_long_lap_steps():
    lap = WallLap()
    assert lap.sparse is None and lap.sparse_spacing > 2 * lap.spacing - .2
