"""Generate front-facing viewpoints from observed surfaces, not hidden targets."""

import math

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt, find_objects, label, uniform_filter
from scipy.sparse.csgraph import dijkstra

from osp_mission.directional_policy import DEFAULT, choose, direction_bins, normals
from osp_mission.policy import graph, wrap

SECTORS=np.arange(-math.pi,math.pi,math.pi/16)
SWEEPS=np.array([0.,math.pi/3,2*math.pi/3,math.pi])
HALF_WIDTHS=SWEEPS/2+1.25/2-np.array([.04,.20,.20,.20])
POSITIVE_STARTS=(SECTORS[None,:]-SWEEPS[:,None]/2+math.pi)%(2*math.pi)-math.pi
NEGATIVE_STARTS=(SECTORS[None,:]+SWEEPS[:,None]/2+math.pi)%(2*math.pi)-math.pi


def make_scans(angles, values, arriving_heading):
    """Offer still views and sweeps, including their actual alignment costs."""
    angular_distance=np.abs((angles[:,None]-SECTORS+math.pi)%(2*math.pi)-math.pi)
    counts=np.sum(values[:,None,None]*(angular_distance[:,None,:]<HALF_WIDTHS[None,:,None]),axis=0)
    positive_turn=np.abs((POSITIVE_STARTS-arriving_heading+math.pi)%(2*math.pi)-math.pi)
    negative_turn=np.abs((NEGATIVE_STARTS-arriving_heading+math.pi)%(2*math.pi)-math.pi)
    turns=np.minimum(positive_turn,negative_turn)
    best=np.argmax(counts-turns*1e-6,axis=1)
    result=[]
    for row,magnitude in enumerate(SWEEPS):
        column=best[row];backwards=negative_turn[row,column]<positive_turn[row,column]
        result.append((float(counts[row,column]),float(NEGATIVE_STARTS[row,column] if backwards else POSITIVE_STARTS[row,column]),
                       float(-magnitude if backwards else magnitude),float(turns[row,column])))
    result.append((float(values.sum()),arriving_heading,2*math.pi,0.))
    return result


def choose_timed(candidates,weights=DEFAULT):
    if not candidates:return None
    def score(candidate):
        surface,frontier,distance,turn,revisit=candidate['features']
        seconds=weights[2]*distance/.26+weights[3]*turn/.5+abs(candidate['scan_angle'])/.5+1.5
        return (weights[0]*surface+weights[1]*frontier)/seconds-weights[4]*revisit*.1
    return max(candidates,key=score)


def surface_weights(occupied, surface, resolution):
    """Keep long connected walls from overwhelming small obstacle faces."""
    objects, _ = label(occupied, structure=np.ones((3, 3)))
    lengths = np.bincount(objects[surface], minlength=objects.max() + 1) * resolution
    weights = np.clip(2.5 / np.maximum(lengths, 1e-6), .25, 1.)
    return weights[objects]


def prior_weights(occupied, surface, resolution, wall_prior=1/3):
    """Allocate search mass to walls and individual observed obstacles.

    All grouping comes from the observed grid. No target, scenario profile,
    carrier identity or completed map is provided to this function.
    """
    objects,_=label(occupied,structure=np.ones((3,3)))
    lengths=np.bincount(objects[surface],minlength=objects.max()+1)*resolution
    walls=[];obstacles=[]
    for ident,slices in enumerate(find_objects(objects),start=1):
        if slices is None or lengths[ident]==0:continue
        span=max(s.stop-s.start for s in slices)*resolution
        (walls if span>=6. else obstacles).append(ident)
    result=np.zeros(len(lengths))
    wall_mass=wall_prior if obstacles else 1.
    object_mass=1-wall_prior if walls else 1.
    if walls:result[walls]=wall_mass/lengths[walls].sum()
    if obstacles:result[obstacles]=object_mass/(len(obstacles)*lengths[obstacles])
    # Preserve a useful feature scale for the existing five learned weights.
    return 100*result[objects]


def options(data, resolution, origin, robot, heading=0., visits=(), viewed=None,
            return_paths=False, face_candidates=True, balanced=True, adaptive_scans=False, directional=True, probabilistic=False):
    free = data == 0
    occupied = data > 50
    unknown = data < 0
    clearance = distance_transform_edt(~occupied) * resolution
    safe = free & (clearance >= .32)
    ry = int((robot[1] - origin[1]) / resolution)
    rx = int((robot[0] - origin[0]) / resolution)
    if not (0 <= ry < data.shape[0] and 0 <= rx < data.shape[1]) or not free[ry, rx]:
        return [], None
    safe[ry, rx] = True
    groups, _ = label(safe)
    safe &= groups == groups[ry, rx]
    if safe.sum() < 4:
        return [], None
    distance, previous = dijkstra(graph(safe, resolution), directed=False,
                                  indices=ry * data.shape[1] + rx, return_predecessors=True)
    distance = distance.reshape(data.shape)
    surface = occupied & binary_dilation(free)
    coverage_shape=(8,)+data.shape if directional else data.shape
    inspected = np.zeros(coverage_shape, dtype=bool) if viewed is None else viewed
    if inspected.shape != coverage_shape:
        raise ValueError('Coverage shape does not match the observation model')
    sy, sx = np.where(surface if directional else surface&~inspected)
    if directional:
        normal_y, normal_x = normals(data, resolution)
    else:
        from osp_mission.surface_endpoint_policy import normals as endpoint_normals
        normal_y, normal_x = endpoint_normals(data, resolution)
    surface_ny,surface_nx=normal_y[sy,sx],normal_x[sy,sx]
    ray_fractions=np.linspace(0,1,max(4,int(3.7/resolution)*2))
    weights = prior_weights(occupied,surface,resolution)[sy,sx] if probabilistic else surface_weights(occupied, surface, resolution)[sy, sx] if balanced else np.ones(len(sy))
    gain = uniform_filter(unknown.astype(float), size=max(3, int(3 / resolution) | 1))
    valid = safe & np.isfinite(distance) & (distance > .45) & ((clearance < 3.2) | (gain > .015))
    valid &= uniform_filter(free.astype(float), size=3) > .65
    ys, xs = np.where(valid)
    spacing = max(1, int(1.0 / resolution))
    buckets = {}
    for y, x in zip(ys, xs):
        key = (y // spacing, x // spacing)
        score = abs(clearance[y, x] - 2.0)
        if key not in buckets or score < buckets[key][0]:
            buckets[key] = (score, y, x)
    candidates = {(int(y), int(x)) for _, y, x in buckets.values()}
    if face_candidates:
        # Preserve close front-facing viewpoints in narrow passages even if a
        # more open point exists in the same spatial bucket. Both normal-axis
        # signs matter for a one-cell thin partition.
        for sign in ((-1, 1) if directional else (1,)):
            for metres in (.5, .8, 1.4, 2.2):
                py = np.rint(sy + sign * normal_y[sy, sx] * metres / resolution).astype(int)
                px = np.rint(sx + sign * normal_x[sy, sx] * metres / resolution).astype(int)
                inside = (py >= 0) & (py < data.shape[0]) & (px >= 0) & (px < data.shape[1])
                py, px = py[inside], px[inside]
                usable = valid[py, px]
                # Half-metre sampling retains distinct faces, with bounded cost.
                bins = {}
                stride = max(1, int(.5 / resolution))
                for y, x in zip(py[usable], px[usable]):
                    key = (int(y // stride), int(x // stride))
                    bins.setdefault(key, (int(y), int(x)))
                candidates.update(bins.values())
    result = []
    for y, x in sorted(candidates):
        px = origin[0] + (x + .5) * resolution
        py = origin[1] + (y + .5) * resolution
        revisit = sum(math.exp(-((px - a) ** 2 + (py - b) ** 2) / .5)
                      for a, b in visits[-100:])
        if revisit > 1.5:
            continue
        distances = np.hypot(sy - y, sx - x)
        incidence = ((y - sy) * surface_ny + (x - sx) * surface_nx) / np.maximum(distances, 1e-9)
        if directional:incidence=np.abs(incidence)
        nearby = (incidence > .6) & (distances * resolution < 3.7 * incidence)
        if directional:nearby &= ~inspected[direction_bins(y-sy,x-sx),sy,sx]
        ty, tx, value = sy[nearby], sx[nearby], weights[nearby]
        angle = 2 * math.pi / 3
        count = 0.
        yaw = math.atan2(py - robot[1], px - robot[0])
        if adaptive_scans:
            predecessor=int(previous[y*data.shape[1]+x])
            arriving_heading=math.atan2(y-predecessor//data.shape[1],x-predecessor%data.shape[1]) if predecessor>=0 else yaw
            departure_turn=abs(wrap(yaw-heading))
        if len(ty):
            lengths = np.hypot(ty - y, tx - x)
            stop = np.maximum(0., 1. - 1.01 / np.maximum(lengths, 1e-9))
            fractions = stop[:, None] * ray_fractions
            ly = np.rint(y + (ty[:, None] - y) * fractions).astype(int)
            lx = np.rint(x + (tx[:, None] - x) * fractions).astype(int)
            visible = np.all(data[ly, lx] == 0, axis=1)
            ty, tx, value = ty[visible], tx[visible], value[visible]
            if len(ty):
                angles = np.arctan2(ty - y, tx - x)
                if adaptive_scans:
                    for scan_gain,start,sweep,alignment in make_scans(angles,value,arriving_heading):
                        if scan_gain==0 and gain[y,x]<.015:continue
                        result.append(dict(x=px,y=py,yaw=wrap(start),scan_angle=sweep,
                            cell=(int(y),int(x)),distance=float(distance[y,x]),
                            features=np.array([scan_gain*resolution,gain[y,x]*5,distance[y,x],departure_turn+alignment,revisit])))
                    continue
                sectors = np.arange(-math.pi, math.pi, math.pi / 8)
                counts = np.sum(value[:, None] * (np.abs(
                    (angles[:, None] - sectors + math.pi) % (2 * math.pi) - math.pi)
                    < math.radians(55)), axis=0)
                best = int(np.argmax(counts))
                count = float(counts[best]) * resolution
                yaw = float(sectors[best]) - math.pi / 3
                if value.sum() > counts[best] * 1.5:
                    count = float(value.sum()) * resolution
                    angle = 2 * math.pi
                    yaw = math.atan2(py - robot[1], px - robot[0])
        if count == 0 and gain[y, x] < .015:
            continue
        if adaptive_scans:
            yaw=arriving_heading;angle=0.
        result.append(dict(x=px, y=py, yaw=wrap(yaw), scan_angle=angle,
                           cell=(int(y), int(x)), distance=float(distance[y, x]),
                           features=np.array([count, gain[y, x] * 5, distance[y, x],
                                              abs(wrap(yaw - heading)), revisit])))
    return result, previous if return_paths else None
