"""Reduced alignment must retain the same continuous observation arc."""

import math
import numpy as np
import pytest

from osp_mission.scan_alignment import plan
from osp_mission.policy import wrap


def test_full_spin_starts_at_arrival_heading_without_an_extra_turn():
    for sweep in (2*math.pi,-2*math.pi):
        heading,actual=plan(2.,-1.,sweep)
        assert heading==pytest.approx(2.) and actual==sweep


def test_partial_spin_may_reverse_but_preserves_every_view_direction():
    rng=np.random.default_rng(880281)
    for _ in range(200):
        arrival,start=rng.uniform(-math.pi,math.pi,2);sweep=rng.uniform(-math.pi,math.pi)
        heading,actual=plan(arrival,start,sweep)
        assert abs(wrap(heading-arrival))<=abs(wrap(start-arrival))+1e-9
        original=np.array([wrap(v) for v in start+np.linspace(0,sweep,101)])
        revised=np.array([wrap(v) for v in heading+np.linspace(0,actual,101)])
        if actual==-sweep:revised=revised[::-1]
        np.testing.assert_allclose(np.cos(original),np.cos(revised),atol=1e-12)
        np.testing.assert_allclose(np.sin(original),np.sin(revised),atol=1e-12)


def test_stationary_view_and_invalid_input():
    assert plan(1.,-.4,0.)==pytest.approx((-.4,0.))
    with pytest.raises(ValueError):plan(float('nan'),0.,1.)
