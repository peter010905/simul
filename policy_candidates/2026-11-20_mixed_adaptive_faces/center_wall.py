"""Choose inspection poses from live SLAM, camera coverage and the Nav2 costmap."""
import math
from collections import deque

import numpy as np
from scipy import ndimage

from osp_mission import _r32_navigation_core as r32
from osp_mission.adaptive_faces import clearance_grid, inspection_pose
from osp_mission.box_views import odom_point
from osp_mission.edge_policy import DIRECTIONS


def partly_mapped_objects(labels, boxes, floor, resolution):
    """Recognize finite occupied returns with observed floor around their flanks."""
    objects = set()
    angles = np.arange(16) * (2 * math.pi / 16)
    directions = np.stack((np.sin(angles), np.cos(angles)), axis=1)
    for ident, box in enumerate(boxes, 1):
        if box is None:
            continue
        extent = max(s.stop-s.start for s in box) * resolution
        if not .35 <= extent <= 2.4:
            continue
        points = np.argwhere(labels[box] == ident)
        if len(points) < 4:
            continue
        points += [s.start for s in box]
        center = points.mean(axis=0)
        radii = extent/2 + np.array([.20, .40, .60])
        ring = np.rint(center + directions[:, None, :] * radii[None, :, None]/resolution).astype(int)
        rows, cols = ring.transpose(2, 0, 1)
        inside = (rows >= 0) & (rows < floor.shape[0]) & (cols >= 0) & (cols < floor.shape[1])
        known = np.zeros(inside.shape, bool)
        known[inside] = floor[rows[inside], cols[inside]]
        open_sides = known.sum(axis=1) >= 2
        if open_sides.sum() < 7:
            continue
        values, axes = np.linalg.eigh(np.cov(points.T) * resolution**2)
        # A short straight fragment of a room wall can have free space past
        # both ends. Require free space across its thickness, or a measured
        # corner with floor on opposed sides of the component.
        opposite = open_sides[:8] & open_sides[8:]
        across = np.abs(directions[:8] @ axes[:, 0]) > .7
        if np.any(opposite & across) or (values[0] >= .065**2 and opposite.any()):
            objects.add(ident)
    return objects


def discovery_frontier(choices, costmap, nav, veto):
    frontier = None
    best_score = -np.inf
    for candidate in choices:
        gain = candidate['features'][1]
        if gain < .15 or candidate.get('stationary') or veto(candidate):
            continue
        cell = costmap.index(candidate['x'], candidate['y'])
        if not costmap.inside(*cell) or not nav[cell]:
            continue
        travel = candidate['distance']
        # Prefer an informative new position over successive tiny steps
        # along the same perimeter. Flow already checked SLAM path safety.
        novelty = candidate['features'][4]
        score = gain/(1. + .18*travel + max(0., 2.-travel) + novelty)
        if score > best_score:
            best_score = score
            frontier = dict(candidate, r32_kind='interior_frontier')
    return frontier


def surface_cells(grid, pose):
    """Separate occupied islands inside explored free space from its exterior."""
    data = grid.cells
    free = (data >= 0) & (data <= 25)
    regions, _ = ndimage.label(free)
    start = grid.index(*pose[:2])
    if not grid.inside(*start) or not regions[start]:
        return {}
    floor = regions == regions[start]
    interior = ndimage.binary_fill_holes(floor) & ~floor
    occupied, _ = ndimage.label(data >= 65, structure=np.ones((3, 3)))
    boxes = ndimage.find_objects(occupied)
    partial = partly_mapped_objects(occupied, boxes, floor, grid.resolution)
    # A wall occupies many cells. Bin samples in metres, independently on each side.
    samples = {}
    for side, (dy, dx) in enumerate(DIRECTIONS):
        adjacent = ndimage.shift(floor, (-dy, -dx), order=0, mode='constant', prefilter=False)
        rows, cols = np.where((occupied > 0) & adjacent)
        for row, col in zip(rows, cols):
            component = occupied[row, col]
            rr, cc = boxes[component-1]
            extent = max(rr.stop-rr.start, cc.stop-cc.start)*grid.resolution
            inner = bool(interior[row, col]) or component in partial
            if extent < (.30 if inner else .75):
                continue
            kind = 'interior' if inner else 'wall'
            stride = max(1, round((.45 if inner else .75)/grid.resolution))
            key = (kind, component, side, row//stride, col//stride)
            samples.setdefault(key, []).append((row, col))
    return samples


class CenterWall:
    def __init__(self):
        self.phase = 'interior'
        self.reason = 'mission_start'
        self.idle_since = None
        self.wall_anchor = None
        self.history = deque(maxlen=200)
        self.counts = {}

    def decide(self, grid, costmap, pose, transform, viewed, elapsed, duration, veto, choices=(), fitted=(), probe_only=False):
        if transform is None or viewed is None or viewed.shape != (4,)+grid.cells.shape:
            return None
        samples = surface_cells(grid, pose)
        reachable, distances = grid.reachable(pose[:2], .32)
        nav = r32.connected_nav_cells(costmap, pose)
        self.counts = {'interior': 0, 'wall': 0}
        clearance = clearance_grid(grid)
        candidates = {'interior': [], 'wall': []}
        fitted_fresh = []
        start = grid.index(*pose[:2])
        if not grid.inside(*start) or clearance[start] < .32:
            fitted = ()
        for center, faces, seen_angles in fitted:
            for side, (face, normal) in enumerate(faces):
                normal_yaw = math.atan2(normal[1], normal[0]) + transform[2]
                if any(math.cos(normal_yaw - angle) >= math.cos(math.radians(35)) for angle in seen_angles):
                    continue
                anchor = odom_point(face, transform)
                attempts = self.attempts('interior', anchor, normal_yaw, .50)
                if len(attempts) >= 2 or (attempts and elapsed-attempts[-1] < 45.):
                    continue
                tangent = np.array([-normal[1], normal[0]])
                half_width = max(abs((p-center) @ tangent) for p, _ in faces)
                fields = dict(r32_kind='interior_surface', box_view_scope='fitted_face',
                              fitted_face=side+1, face_inspection=True, stationary=False, scan_angle=0.,
                              normal_odom=normal_yaw, surface_odom=anchor.tolist(), surface_map=face.tolist(),
                              features=np.zeros(5), previous_attempts=len(attempts))
                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,
                                       half_width, pose, fields, veto, visibility_offset=.12)
                if view:
                    candidates['interior'].append(view)
                    if not attempts:
                        fitted_fresh.append(view)
        if fitted_fresh:
            self.counts = {'interior': len(fitted_fresh), 'wall': None}
            if not probe_only:
                self.phase, self.reason = 'interior', 'unseen_fitted_face'
                self.wall_anchor = self.idle_since = None
            return min(fitted_fresh,key=lambda item:item[0])[1]
        for (kind, _, side, _, _), cells in samples.items():
            if probe_only and kind != 'interior':
                continue
            # Keep one-off scan speckles out of each inspection target.
            if len(cells)*grid.resolution < .15:
                continue
            remaining = [(r,c) for r,c in cells if not viewed[side,r,c]]
            if len(remaining)*grid.resolution < .15:
                continue
            row,col = remaining[len(remaining)//2]
            dy,dx = DIRECTIONS[side]
            face = grid.world(row+.5*dy, col+.5*dx)
            normal = (grid.world(row+dy,col+dx)-grid.world(row,col))/grid.resolution
            normal_yaw = math.atan2(normal[1],normal[0])+transform[2]
            anchor = odom_point(face,transform)
            radius = .5 if kind == 'interior' else .85
            attempts = self.attempts(kind, anchor, normal_yaw, radius)
            if len(attempts)>=2 or (attempts and elapsed-attempts[-1]<45.):
                continue
            if kind == 'interior':
                fields = dict(r32_kind='interior_surface', surface_odom=anchor.tolist(),
                              normal_odom=normal_yaw, surface_map=face.tolist(), previous_attempts=len(attempts),
                              features=np.zeros(5), stationary=False, face_inspection=True, scan_angle=math.radians(36))
                view = inspection_pose(grid, costmap, nav, distances, clearance, face, normal,
                                       .5*len(cells)*grid.resolution, pose, fields, veto)
                if view:
                    candidates[kind].append(view)
                continue
            tangent = np.array([-normal[1],normal[0]])
            best = None
            for stand in (.67,.88,1.15):
                for shift in (0.,-.28,.28):
                    point = face+normal*stand+tangent*shift
                    cell = grid.index(*point)
                    if not grid.inside(*cell) or not reachable[cell] or not np.isfinite(distances[cell]):
                        continue
                    nc = costmap.index(*point)
                    if not costmap.inside(*nc) or not nav[nc] or not grid.line_clear(point,face+.06*normal):
                        continue
                    heading = math.atan2(face[1]-point[1],face[0]-point[0])
                    sweep = math.radians(36)
                    goal = dict(x=float(point[0]), y=float(point[1]), yaw=heading-sweep/2,
                                scan_angle=sweep, face_inspection=True, stationary=False,
                                distance=float(np.linalg.norm(point-pose[:2])), features=np.zeros(5),
                                r32_kind=kind+'_surface', surface_odom=anchor.tolist(),
                                normal_odom=normal_yaw, surface_map=face.tolist(),
                                previous_attempts=len(attempts))
                    if veto(goal):
                        continue
                    score = distances[cell] + .5*abs(stand-.67) + .3*abs(shift) + 2.*len(attempts)
                    if kind=='wall' and self.wall_anchor is not None:
                        last, last_normal = self.wall_anchor
                        step = anchor-last
                        forward = np.array([-math.sin(last_normal), math.cos(last_normal)])
                        score += .55*np.linalg.norm(step) + 2.5*max(0.,-float(step@forward))
                    if best is None or score<best[0]:
                        best = score,goal
            if best is not None:
                candidates[kind].append(best)
        self.counts = {k:len(v) for k,v in candidates.items()}
        fresh = [item for item in candidates['interior'] if not item[1]['previous_attempts']]
        if probe_only:
            return min(fresh,key=lambda item:item[0])[1] if fresh else None
        wall_reserve = min(.15*duration, 12. + 5.*self.counts['wall'])
        frontier = None if fresh else discovery_frontier(choices, costmap, nav, veto)
        if fresh or (frontier is not None and duration-elapsed > wall_reserve):
            if self.phase == 'wall':
                self.wall_anchor = None
            self.phase = 'interior'
            self.idle_since = None
            if fresh:
                self.reason = 'unseen_reachable_object'
                return min(fresh,key=lambda item:item[0])[1]
            if not candidates['interior']:
                self.reason = 'explore_open_frontier'
                return frontier
        if self.phase == 'interior':
            # Reserve a small final wall pass only when real wall viewpoints
            # exist. A longer mapped perimeter merits more of the remaining time.
            if candidates['wall'] and duration-elapsed <= wall_reserve:
                self.phase,self.reason = 'wall','remaining_time_for_mapped_walls'
            elif frontier is None and elapsed >= duration*.55 and candidates['wall']:
                self.phase,self.reason = 'wall','no_untried_reachable_interior_view'
            elif candidates['interior']:
                self.idle_since = None
            elif self.idle_since is None:
                self.idle_since = elapsed
            elif (candidates['wall'] and elapsed-self.idle_since>=min(35.,duration*.13)
                  and elapsed>=min(60.,duration*.22)):
                self.phase,self.reason = 'wall','no_reachable_interior_views'
        best = min(candidates[self.phase],key=lambda x:x[0],default=None)
        return best[1] if best else None

    def attempts(self, kind, anchor, normal_yaw, radius):
        return [t for k,p,a,t in self.history if k == kind and
                np.linalg.norm(anchor-p)<radius and math.cos(normal_yaw-a)>.70]

    def dispatched(self, goal, elapsed):
        if 'surface_odom' in goal:
            anchor = np.asarray(goal['surface_odom'])
            normal = goal['normal_odom']
            self.history.append((self.phase,anchor,normal,elapsed))
            if self.phase=='wall':
                self.wall_anchor = anchor,normal
