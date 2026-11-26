"""Schedule safe viewpoints across regions using SLAM and processed RGB coverage."""
from collections import defaultdict, deque
import math

import numpy as np

REGION_RADIUS = 2.5
LOW_GAIN_M = .25


def coverage_witnesses(samples, viewed, grid, transform):
    """Store currently unseen interior face cells in odom, including surface normal."""
    rows = [(side, r, c) for (kind, _, side, _, _), cells in samples.items()
            if kind == 'interior' for r, c in cells if not viewed[side, r, c]]
    if not rows:
        return np.empty((0, 3))
    sides, r, c = np.array(rows).T
    gx, gy, ga = grid.origin
    tx, ty, ta = transform
    angle = ga + ta
    x, y = (c+.5)*grid.resolution, (r+.5)*grid.resolution
    ox = tx + math.cos(ta)*gx - math.sin(ta)*gy
    oy = ty + math.sin(ta)*gx + math.cos(ta)*gy
    return np.column_stack((ox+math.cos(angle)*x-math.sin(angle)*y,
                            oy+math.sin(angle)*x+math.cos(angle)*y, sides))


def measure(snapshot, grid, transform, viewed):
    points, old_angle = snapshot
    tx, ty, ta = transform
    gx, gy, ga = grid.origin
    if not len(points) or math.cos(old_angle-ta-ga) < math.cos(math.radians(15)):
        return None
    # Reproject old witnesses rather than subtracting total coverage of a changing map.
    x, y = points[:, 0]-tx, points[:, 1]-ty
    mx = math.cos(ta)*x+math.sin(ta)*y-gx
    my = -math.sin(ta)*x+math.cos(ta)*y-gy
    c = np.floor((math.cos(ga)*mx+math.sin(ga)*my)/grid.resolution).astype(int)
    r = np.floor((-math.sin(ga)*mx+math.cos(ga)*my)/grid.resolution).astype(int)
    valid = (r >= 0) & (c >= 0) & (r < grid.cells.shape[0]) & (c < grid.cells.shape[1])
    valid[valid] &= grid.cells[r[valid], c[valid]] >= 65
    if valid.sum() < max(3, .55*len(points)):
        return None
    return float(viewed[points[valid, 2].astype(int), r[valid], c[valid]].sum()*grid.resolution)


def strips(walls):
    """Contiguous, same-component and same-facing samples. Bound each sweep in metres."""
    lines = defaultdict(list)
    for _, goal in walls:
        component, side = goal['wall_line']
        lines[component, side].append(goal)
    for goals in lines.values():
        normal = goals[0]['normal_odom']
        tangent = (-math.sin(normal), math.cos(normal))
        def along(g):
            p = g['surface_odom']
            return p[0]*tangent[0]+p[1]*tangent[1]
        group = []
        for goal in sorted(goals, key=along):
            point = goal['surface_odom']
            if group and (math.dist(point, group[-1]['surface_odom']) > 1.35
                          or along(goal)-along(group[0]) > 4.5
                          or abs((point[0]-group[0]['surface_odom'][0])*math.cos(normal)
                                +(point[1]-group[0]['surface_odom'][1])*math.sin(normal)) > .45):
                yield group
                group = []
            group.append(goal)
        if group:
            yield group


class RegionWall:
    def __init__(self):
        self.pending = None
        self.low = deque(maxlen=16)
        self.cool = []
        self.visits = deque(maxlen=48)
        self.sweep = None
        self.next_wall = 90.
        self.deadline = math.inf
        self.can_interrupt = False
        self.audit = {}

    def observe(self, samples, grid, transform, viewed, elapsed, fresh):
        self.witnesses = coverage_witnesses(samples, viewed, grid, transform)
        self.angle = grid.origin[2]+transform[2]
        self.cool = [item for item in self.cool if elapsed < item[1]]
        self.audit = {}
        if not self.pending:
            return
        point, snapshot, start = self.pending
        self.pending = None
        gain = measure(snapshot, grid, transform, viewed)
        self.audit = dict(region_anchor=point, new_face_length_m=gain, interval_s=elapsed-start)
        if gain is None or elapsed-start < 2:
            return
        if gain >= LOW_GAIN_M:
            self.low = deque((v for v in self.low if math.dist(v[0], point)>REGION_RADIUS), maxlen=16)
            return
        self.low.append((point, elapsed))
        if sum(math.dist(p, point)<=REGION_RADIUS and elapsed-t<=95 for p,t in self.low) >= 2:
            known = [g for _,g in fresh if 'fitted_face' in g and math.dist(g['surface_odom'],point)<=REGION_RADIUS]
            self.cool.append((point, elapsed+75., [(g['surface_odom'],g['normal_odom']) for g in known]))
            self.audit['low_yield_exit'] = True

    def deferred(self, goal):
        p = goal['surface_odom']
        for center, _, faces in self.cool:
            if math.dist(center, p) <= REGION_RADIUS:
                # A newly revealed fitted face can reopen this area.
                if 'fitted_face' not in goal or any(math.dist(a,p)<.42 and
                        math.cos(y-goal['normal_odom'])>.82 for a,y in faces):
                    return True
        return False

    def choose(self, walls, elapsed, duration, weak=None):
        self.deadline = math.inf
        best = None
        for group in strips(walls):
            center = np.mean([g['surface_odom'] for g in group], axis=0)
            normal = group[0]['normal_odom']
            length = sum(g['unseen_wall_m'] for g in group)
            entry = min(group, key=lambda g:g['wall_walk_m'])
            travel = entry['wall_walk_m']/.20
            sweep_s = 7.+length/.20
            repeated = sum(elapsed-t < 180 and math.dist(p,center)<3. for p,t in self.visits)
            active = self.sweep and elapsed < self.sweep[2] and math.dist(center,self.sweep[0])<2.8 and math.cos(normal-self.sweep[1])>.82
            # Stick to the chosen strip briefly, but leave enough time to reach it.
            value = length/(12.+travel+sweep_s)/(1.+.60*repeated)
            if active:
                value *= 2.8
            if weak and any(math.dist(weak['surface_odom'],g['surface_odom'])<.2 for g in group):
                entry = weak
                value *= 2.
            if best is None or value>best[0]:
                best = value, entry, center.tolist(), length, entry['wall_walk_m']/.20+sweep_s+15., bool(active)
        if best is None:
            return None
        _, goal, center, length, reserve, active = best
        # Re-evaluate a long general transit once a mapped strip is due, even mid-route.
        self.deadline = min(self.next_wall, duration-reserve)
        if not (active or elapsed >= self.deadline or self.cool or weak):
            return None
        reason = 'continue_wall_strip' if active else ('region_low_yield' if self.cool else 'wall_strip_due')
        self.audit.update(wall_strip_m=round(length,2), wall_eta_budget_s=round(reserve,1))
        return dict(goal, region_wall=True, strip_center=center, strip_reason=reason)

    def dispatched(self, goal, elapsed):
        self.can_interrupt = goal.get('r32_kind') in ('interior_surface','interior_frontier') and self.deadline>elapsed
        self.pending = None
        if goal.get('region_wall'):
            center = goal['strip_center']
            if goal['strip_reason']!='continue_wall_strip':
                self.sweep = center, goal['normal_odom'], elapsed+32.
            self.visits.append((goal['surface_odom'],elapsed))
            self.next_wall = elapsed+85.
        if goal.get('r32_kind')=='interior_surface' and hasattr(self,'witnesses'):
            point = list(goal['surface_odom'])
            ids = np.linalg.norm(self.witnesses[:,:2]-point,axis=1)<=REGION_RADIUS
            self.pending = point, (self.witnesses[ids],self.angle), elapsed

    def due_in_transit(self, elapsed):
        if self.can_interrupt and elapsed>=self.deadline:
            self.can_interrupt=False
            return True
        return False
