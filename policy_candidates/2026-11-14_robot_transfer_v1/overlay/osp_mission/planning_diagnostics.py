"""Explain a lack of goals using policy observations only."""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label
from osp_mission.escape_connectivity import connect_start


def explain(data, resolution, origin, robot, before, after):
    y = math.floor((robot[1]-origin[1])/resolution)
    x = math.floor((robot[0]-origin[0])/resolution)
    result = dict(raw_candidates=before, eligible_candidates=after, start_cell=[y, x])
    if after:
        return dict(result, reason='eligible_goals')
    if before:
        return dict(result, reason='viewpoint_retry_cooldown')
    if not (0 <= y < data.shape[0] and 0 <= x < data.shape[1]):
        return dict(result, reason='start_outside_map')
    if data[y, x] != 0:
        return dict(result, reason='start_unknown' if data[y, x] < 0 else 'start_not_free')
    clearance = distance_transform_edt(data <= 50)*resolution
    safe = connect_start(data, resolution, (y, x), (data == 0) & (clearance >= .32))
    groups, _ = label(safe)
    count = int(np.count_nonzero(groups == groups[y, x])) if safe[y, x] else 0
    return dict(result, start_clearance_m=float(clearance[y, x]), reachable_cells=count,
                reason='start_clearance_disconnected' if count < 4 else 'no_informative_viewpoints')
