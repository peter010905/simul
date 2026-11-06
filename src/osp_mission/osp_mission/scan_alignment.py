"""Preserve an observation arc while minimizing its initial alignment turn."""

import math

from osp_mission.policy import wrap


def plan(arrival_heading, start_heading, sweep):
    if not all(math.isfinite(v) for v in (arrival_heading,start_heading,sweep)):
        raise ValueError('Observation angles must be finite')
    if abs(sweep)>=2*math.pi-1e-9:
        return wrap(arrival_heading),sweep
    reverse_start=wrap(start_heading+sweep)
    if abs(wrap(reverse_start-arrival_heading))+1e-9<abs(wrap(start_heading-arrival_heading)):
        return reverse_start,-sweep
    return wrap(start_heading),sweep
