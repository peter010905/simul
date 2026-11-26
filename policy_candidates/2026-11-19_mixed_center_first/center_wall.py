"""Choose inspection poses from live SLAM, camera coverage and the Nav2 costmap."""
import math
from collections import deque

import numpy as np
from scipy import ndimage

from osp_mission import _r32_navigation_core as r32
from osp_mission.box_views import odom_point
from osp_mission.edge_policy import DIRECTIONS


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
    # A wall occupies many cells. Bin samples in metres, independently on each side.
    samples = {}
    for side, (dy, dx) in enumerate(DIRECTIONS):
        adjacent = ndimage.shift(floor, (-dy, -dx), order=0, mode='constant', prefilter=False)
        rows, cols = np.where((occupied > 0) & adjacent)
        for row, col in zip(rows, cols):
            component = occupied[row, col]
            rr, cc = boxes[component-1]
            extent = max(rr.stop-rr.start, cc.stop-cc.start)*grid.resolution
            inner = bool(interior[row, col])
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

    def decide(self, grid, costmap, pose, transform, viewed, elapsed, duration, veto):
        if transform is None or viewed is None or viewed.shape != (4,)+grid.cells.shape:
            return None
        samples = surface_cells(grid, pose)
        reachable, distances = grid.reachable(pose[:2], .32)
        nav = r32.connected_nav_cells(costmap, pose)
        self.counts = {'interior': 0, 'wall': 0}
        candidates = {'interior': [], 'wall': []}
        for (kind, _, side, _, _), cells in samples.items():
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
            attempts = [t for k,p,a,t in self.history if k == kind and
                        np.linalg.norm(anchor-p)<radius and math.cos(normal_yaw-a)>.70]
            if len(attempts)>=2 or (attempts and elapsed-attempts[-1]<45.):
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
        if self.phase == 'interior':
            fresh = [item for item in candidates['interior'] if not item[1]['previous_attempts']]
            # Reserve a small final wall pass only when real wall viewpoints
            # exist. A longer mapped perimeter merits more of the remaining time.
            wall_reserve = min(.15*duration, 12. + 5.*self.counts['wall'])
            if candidates['wall'] and duration-elapsed <= wall_reserve:
                self.phase,self.reason = 'wall','remaining_time_for_mapped_walls'
            elif fresh:
                candidates['interior'] = fresh
                self.idle_since = None
            elif elapsed >= duration*.55 and candidates['wall']:
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

    def dispatched(self, goal, elapsed):
        if 'surface_odom' in goal:
            anchor = np.asarray(goal['surface_odom'])
            normal = goal['normal_odom']
            self.history.append((self.phase,anchor,normal,elapsed))
            if self.phase=='wall':
                self.wall_anchor = anchor,normal
