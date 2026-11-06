"""Choose an observable goal that can be completed before the mission deadline."""

import math

from osp_mission.surface_endpoint_policy import choose as choose_surface


def choose(candidates, weights, remaining_seconds):
    if not math.isfinite(remaining_seconds):
        return choose_surface(candidates, weights)
    feasible=[]
    for candidate in candidates:
        surface,frontier,distance,turn,revisit=candidate['features']
        # Conservative nominal motion rates, independent of hidden targets.
        required=distance/.20+turn/.45+abs(candidate.get('scan_angle',2*math.pi/3))/.45+3.5
        if required>remaining_seconds:
            continue
        updated=dict(candidate);updated['features']=candidate['features'].copy()
        # With little time left, prioritize inspecting already reachable faces.
        updated['features'][1]*=min(1.,max(0.,remaining_seconds)/90.)
        feasible.append(updated)
    return choose_surface(feasible, weights)
