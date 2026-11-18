"""Choose camera views with explicit coverage overlap and short route lookahead.

Inputs are the observed occupancy map and confirmed face observations. Hidden
marker locations, the scenario configuration and the completed map are absent.
"""

import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label, uniform_filter
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from osp_mission.edge_policy import boundary_faces, visible
from osp_mission.face_policy import make_scans
from osp_mission.policy import graph, wrap
from osp_mission.surface_endpoint_policy import DEFAULT


def scan_mask(bearings, yaw, sweep):
    """Conservative readable angular interval, leaving margin for three frames."""
    if abs(sweep) >= 2 * math.pi - 1e-6:
        return np.ones(len(bearings), dtype=bool)
    centre = yaw + sweep / 2
    half = abs(sweep) / 2 + 1.25 / 2 - (.04 if abs(sweep) < 1e-6 else .20)
    return np.abs((bearings - centre + math.pi) % (2 * math.pi) - math.pi) < half


def options(data, resolution, origin, robot, heading=0., visits=(), viewed=None,
            return_paths=False, point_sampler=None):
    free, occupied, unknown = data == 0, data > 50, data < 0
    clearance = distance_transform_edt(~occupied) * resolution
    safe = free & (clearance >= .32)
    ry, rx = int((robot[1] - origin[1]) / resolution), int((robot[0] - origin[0]) / resolution)
    if not (0 <= ry < data.shape[0] and 0 <= rx < data.shape[1]) or not free[ry, rx]:
        return [], None
    safe[ry, rx] = True
    groups, _ = label(safe)
    safe &= groups == groups[ry, rx]
    if safe.sum() < 4:
        return [], None
    network = graph(safe, resolution)
    distances, previous = dijkstra(network, directed=False, indices=ry * data.shape[1] + rx,
                                   return_predecessors=True)
    distances = distances.reshape(data.shape)
    faces = boundary_faces(data, resolution)
    state = np.zeros((4,) + data.shape, dtype=bool) if viewed is None else viewed
    if state.shape != (4,) + data.shape:
        raise ValueError('Four boundary-face observation states are required')
    all_faces = faces
    unseen = ~state[faces['side'], faces['y'], faces['x']]
    faces = {k: v[unseen] for k, v in faces.items()}
    gain = uniform_filter(unknown.astype(float), size=max(3, int(3 / resolution) | 1))
    valid = safe & np.isfinite(distances) & ((clearance < 3.2) | (gain > .015))
    valid &= uniform_filter(free.astype(float), size=3) > .65
    if point_sampler is None:
        buckets = {}
        spacing = max(1, int(1 / resolution))
        for y, x in zip(*np.where(valid & (distances > .45))):
            key = (int(y // spacing), int(x // spacing))
            score = abs(clearance[y, x] - 2.)
            if key not in buckets or score < buckets[key][0]:
                buckets[key] = (score, int(y), int(x))
        points = [(y, x) for _, y, x in buckets.values()]
    else:
        points = point_sampler(data, resolution, origin, robot, safe, distances, all_faces, state)
    # Permit looking at a different face before leaving a visited location.
    if safe[ry, rx]:
        points.append((ry, rx))
    candidates = []
    for y, x in points:
        inplace = (y, x) == (ry, rx)
        px = robot[0] if inplace else origin[0] + (x + .5) * resolution
        py = robot[1] if inplace else origin[1] + (y + .5) * resolution
        grid_y, grid_x = (py - origin[1]) / resolution, (px - origin[0]) / resolution
        revisit = sum(math.exp(-((px - a) ** 2 + (py - b) ** 2) / .5) for a, b in visits[-100:])
        dy, dx = grid_y - faces['py'], grid_x - faces['px']
        lengths = np.hypot(dy, dx)
        incidence = (dy * faces['ny'] + dx * faces['nx']) / np.maximum(lengths, 1e-9)
        ids = np.flatnonzero((incidence > .6) & (lengths * resolution > .25)
                             & (lengths * resolution < 3.7 * incidence))
        ids = ids[visible(data, grid_y, grid_x, faces['py'][ids], faces['px'][ids])]
        departure = heading if inplace else math.atan2(py - robot[1], px - robot[0])
        predecessor = int(previous[y * data.shape[1] + x])
        arrival = heading if inplace or predecessor < 0 else math.atan2(y - predecessor // data.shape[1], x - predecessor % data.shape[1])
        bearings = np.arctan2(-dy[ids], -dx[ids])
        scans = make_scans(bearings, np.ones(len(ids)), arrival) if len(ids) else [(0., arrival, 0., 0.)]
        for _, yaw, sweep, alignment in scans:
            covered = ids[scan_mask(bearings, yaw, sweep)]
            frontier = 0. if inplace else float(gain[y, x] * 5)
            if not len(covered) and frontier < .075:
                continue
            candidates.append(dict(x=px, y=py, yaw=wrap(yaw), scan_angle=sweep,
                                   cell=(y, x), distance=float(distances[y, x]),
                                   features=np.array([len(covered) * resolution, frontier, distances[y, x],
                                                      abs(wrap(departure - heading)) + alignment, revisit]),
                                   face_ids=covered))
    if candidates:
        row_ids, col_ids = [], []
        for i, c in enumerate(candidates):
            row_ids.extend([i] * len(c['face_ids']))
            col_ids.extend(c['face_ids'])
        coverage = csr_matrix((np.ones(len(row_ids)), (row_ids, col_ids)),
                              shape=(len(candidates), len(unseen.nonzero()[0])))
        context = dict(network=network, coverage=coverage, resolution=resolution,
                       cells=np.array([c['cell'][0] * data.shape[1] + c['cell'][1] for c in candidates]))
        for c in candidates:
            c['_route_context'] = context
    return candidates, previous if return_paths else None


def scores(candidates, weights):
    features = np.array([c['features'] for c in candidates])
    surface, frontier, distance, turn, revisit = features.T
    reward = weights[0] * surface + weights[1] * frontier
    seconds = weights[2] * distance / .26 + weights[3] * turn / .5
    seconds += np.array([abs(c['scan_angle']) / .5 + 1.5 for c in candidates])
    penalty = weights[4] * revisit * .1
    return reward, seconds, penalty


def choose_greedy(candidates, weights=DEFAULT):
    if not candidates:
        return None
    reward, seconds, penalty = scores(candidates, weights)
    return candidates[int(np.argmax(reward / seconds - penalty))]


def choose(candidates, weights=DEFAULT):
    """Compare the best first views using marginal coverage at a second stop."""
    if not candidates:
        return None
    reward, seconds, penalty = scores(candidates, weights)
    immediate = reward / seconds - penalty
    # Retain different places rather than allowing scan variants at one place
    # to consume the entire lookahead budget.
    first = []
    counts = {}
    for index in np.argsort(-immediate):
        cell = candidates[index]['cell']
        if counts.get(cell, 0) < 2:
            first.append(int(index))
            counts[cell] = counts.get(cell, 0) + 1
        if len(first) >= 16:
            break
    context = candidates[0]['_route_context']
    unique_cells, inverse = np.unique(context['cells'][first], return_inverse=True)
    future_distances = dijkstra(context['network'], directed=False,
                                indices=unique_cells)[:, context['cells']][inverse]
    coverage = context['coverage']
    overlap = (coverage[first] @ coverage.T).toarray() * context['resolution']
    positions = np.array([[c['x'], c['y']] for c in candidates])
    yaws = np.array([c['yaw'] for c in candidates])
    sweeps = np.array([c['scan_angle'] for c in candidates])
    frontiers = np.array([c['features'][1] for c in candidates])
    best_score, best = -float('inf'), candidates[int(np.argmax(immediate))]
    for row, index in enumerate(first):
        offset = positions - positions[index]
        bearings = np.arctan2(offset[:, 1], offset[:, 0])
        end_yaw = yaws[index] + sweeps[index]
        turn = np.abs((bearings - end_yaw + math.pi) % (2 * math.pi) - math.pi)
        turn += np.abs((yaws - bearings + math.pi) % (2 * math.pi) - math.pi)
        cost = weights[2] * future_distances[row] / .26 + weights[3] * turn / .5 + np.abs(sweeps) / .5 + 1.5
        separation = np.linalg.norm(offset, axis=1)
        marginal = np.maximum(0., reward - weights[0] * overlap[row])
        # Nearby unknown-map gain is highly correlated and is not counted twice.
        marginal -= weights[1] * frontiers * np.exp(-separation ** 2 / 9.)
        marginal = np.maximum(0., marginal)
        eligible = (separation > .75) & np.isfinite(cost)
        pair = np.where(eligible, (reward[index] + .7 * marginal) / (seconds[index] + .7 * cost)
                        - .5 * (penalty[index] + penalty), -np.inf)
        score = .5 * immediate[index] + .5 * max(immediate[index], float(np.max(pair)))
        if score > best_score:
            best_score, best = score, candidates[index]
    return best
