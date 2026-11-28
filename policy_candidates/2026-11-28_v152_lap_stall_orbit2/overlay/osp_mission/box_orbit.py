""".70: finish every side of a tracked box, including sides the map does not show yet.

Gazebo (scene16-room-slam1128, 11 runs of .69) showed that box sides were rarely all seen:
56/131 viewable faces, every face of a box in 4/33 boxes. Two of the three failures were a
marker side that was never even a target. Face tasks exist only for mapped surfaces (four-face
fits only within 2.7 m of the robot), and one box gets 55 s, then a 90 s cooldown. So the side
facing away from every route never became a task.

Here each confirmed box keeps the bearings from which the camera actually looked at it
(robot 0.45-2.0 m away, heading at the box, line of sight clear). While the largest gap between
those bearings exceeds 100 deg, some side is not seen within 50 deg of its normal. A viewpoint is
made on the far side of that gap from the box centre, whether or not that side is mapped.
- The box being served is finished first; then the most-seen box within a short route.
- At most two attempts per bearing sector and 120 s per box; a box with a registered marker is done.
- A sector without a safe viewpoint counts as covered, so a box against a wall can finish.

.74: boxes between desks were missed with the robot 1.3-1.6 m away but never in front of the
marker side. A confirmed box not yet seen by the camera is served too (its nearest side first,
route <= 4 m), and viewpoints may stand 0.55 m away and up to 50 deg off the side normal.
"""
import math

import numpy as np

SECTORS = 8
GAP_DONE = math.radians(100)
VIEW_RANGE_M = (.45, 2.0)
HEADING_TOL = math.radians(28)
SERVICE_S = 120.
MAX_ATTEMPTS = 2
MAX_WALK_M = 5.0
SAME_BOX_M = .40
STANDS_M = (.55, .75, .95, 1.15, 1.35)
OFFSETS = (0., math.radians(-18), math.radians(18), math.radians(-34), math.radians(34),
           math.radians(-50), math.radians(50))
UNSEEN_MAX_WALK_M = 4.0


def wrap(angle):
    return math.remainder(angle, math.tau)


def to_grid(point, transform):
    """Odom point to the grid (map) frame; inverse of center_wall.odom_point."""
    x, y, a = transform
    c, s = math.cos(a), math.sin(a)
    dx, dy = point[0]-x, point[1]-y
    return np.array([c*dx+s*dy, -s*dx+c*dy])


def to_odom(point, transform):
    x, y, a = transform
    c, s = math.cos(a), math.sin(a)
    return np.array([x+c*point[0]-s*point[1], y+s*point[0]+c*point[1]])


def surface_along(grid, center, direction, limit=.8):
    """Distance from the box centre to its measured surface along direction (None if unmeasured)."""
    last = None
    for d in np.arange(0., limit, grid.resolution/2):
        cell = grid.index(*(center + d*direction))
        if not grid.inside(*cell):
            break
        value = grid.cells[cell]
        if value >= 65:
            last = d
        elif last is not None and 0 <= value <= 25:
            break
    return None if last is None else last + grid.resolution


def largest_gap(bearings):
    """(gap, middle bearing) of the largest angular gap; a full circle when nothing was seen."""
    if not bearings:
        return math.tau, None
    ordered = sorted(b % math.tau for b in bearings)
    gaps = [((ordered[(i+1) % len(ordered)] - b) % math.tau or math.tau, b) for i, b in enumerate(ordered)]
    gap, start = max(gaps)
    return gap, start + gap/2


class BoxOrbit:
    def __init__(self):
        self.boxes = []          # dicts: center (odom), bearings (odom), attempts{sector}, started, blocked[]
        self.active = None
        self.last_choice = None

    def box(self, center):
        center = np.asarray(center, dtype=float)
        for item in self.boxes:
            if np.linalg.norm(item['center']-center) < SAME_BOX_M:
                item['center'] = .8*item['center'] + .2*center
                return item
        item = dict(center=center, bearings=[], attempts={}, started=None, blocked=[])
        self.boxes.append(item)
        return item

    def observe(self, centers, grid, pose, transform):
        """Record the bearing of every box the camera is looking at from this pose."""
        if transform is None:
            return
        robot = np.asarray(pose[:2], dtype=float)
        for center in centers:
            item = self.box(center)
            middle = to_grid(item['center'], transform)
            ray = robot - middle
            distance = float(np.linalg.norm(ray))
            if not VIEW_RANGE_M[0] <= distance <= VIEW_RANGE_M[1]:
                continue
            if abs(wrap(pose[2] - math.atan2(-ray[1], -ray[0]))) > HEADING_TOL:
                continue
            direction = ray / distance
            reach = surface_along(grid, middle, direction) or .30
            if reach >= distance - .2 or not grid.line_clear(robot, middle + reach*direction, allow_unknown=True):
                continue
            bearing = math.atan2(ray[1], ray[0]) + transform[2]
            if not any(abs(wrap(bearing-b)) < math.radians(10) for b in item['bearings']):
                item['bearings'].append(bearing)

    def gap(self, item):
        return largest_gap(item['bearings'] + item['blocked'])

    @staticmethod
    def sector(bearing):
        return int(((bearing % math.tau) / math.tau) * SECTORS) % SECTORS

    def pending(self, item, completed, elapsed):
        if any(np.linalg.norm(item['center']-np.asarray(c)) < SAME_BOX_M for c in completed):
            return False
        if item['started'] is not None and elapsed - item['started'] > SERVICE_S:
            return False
        return self.gap(item)[0] > GAP_DONE

    def viewpoint(self, item, bearing, grid, costmap, nav, distances, clearance, transform, veto, pose,
                  max_walk=MAX_WALK_M):
        middle = to_grid(item['center'], transform)
        direction = np.array([math.cos(bearing-transform[2]), math.sin(bearing-transform[2])])
        reach = surface_along(grid, middle, direction) or .35
        face = middle + reach*direction
        best = None
        for offset in OFFSETS:
            ray = np.array([math.cos(bearing-transform[2]+offset), math.sin(bearing-transform[2]+offset)])
            for stand in STANDS_M:
                point = face + stand*ray
                cell, nc = grid.index(*point), costmap.index(*point)
                if not grid.inside(*cell) or not costmap.inside(*nc) or not nav[nc]:
                    continue
                walk = float(distances[cell])
                if not np.isfinite(walk) or walk > max_walk or clearance[cell] < .32:
                    continue
                if not grid.line_clear(point, face + .06*direction, allow_unknown=True):
                    continue
                score = .24*walk + .6*abs(stand-.95) + .8*abs(offset)
                if best is None or score < best[0]:
                    best = (score, point, walk)
        if best is None:
            return None
        _, point, walk = best
        sweep = math.radians(24)
        normal_yaw = bearing
        goal = dict(x=float(point[0]), y=float(point[1]),
                    yaw=math.atan2(face[1]-point[1], face[0]-point[0]) - sweep/2,
                    scan_angle=sweep, face_inspection=True, stationary=False, features=np.zeros(5),
                    distance=float(np.linalg.norm(point-np.asarray(pose[:2]))),
                    r32_kind='interior_surface', box_view_scope='box_orbit', box_orbit=True,
                    normal_odom=normal_yaw, surface_odom=to_odom(face, transform).tolist(),
                    surface_map=face.tolist(), batch_box=tuple(item['center']), previous_attempts=0,
                    recovery_geometry=(to_odom(face, transform).tolist(), normal_yaw, .3, .12),
                    box_orbit_gap_deg=round(math.degrees(self.gap(item)[0])))
        if veto(goal):
            return None
        return walk, goal

    def choose(self, centers, completed, active_box, grid, costmap, nav, distances, clearance,
               transform, veto, pose, elapsed):
        if transform is None:
            return None
        for center in centers:
            self.box(center)
        items = [item for item in self.boxes if self.pending(item, completed, elapsed)]
        def order(item):
            serving = active_box is not None and np.linalg.norm(item['center']-np.asarray(active_box)) < SAME_BOX_M
            started = item['started'] is not None
            robot_odom = to_odom(np.asarray(pose[:2]), transform)
            return (not serving, not started, -len(item['bearings']),
                    float(np.linalg.norm(item['center']-robot_odom)))
        for item in sorted(items, key=order):
            for _ in range(SECTORS):
                gap, bearing = self.gap(item)
                if gap <= GAP_DONE:
                    break
                if bearing is None:
                    # Never seen by the camera: the side facing the robot first.
                    robot_odom = to_odom(np.asarray(pose[:2]), transform)
                    bearing = math.atan2(robot_odom[1]-item['center'][1], robot_odom[0]-item['center'][0])
                sector = self.sector(bearing)
                if item['attempts'].get(sector, 0) >= MAX_ATTEMPTS:
                    item['blocked'].append(bearing)
                    continue
                view = self.viewpoint(item, bearing, grid, costmap, nav, distances, clearance,
                                      transform, veto, pose,
                                      MAX_WALK_M if item['bearings'] else UNSEEN_MAX_WALK_M)
                if view is None:
                    item['blocked'].append(bearing)
                    continue
                self.last_choice = (item, sector)
                return view[1]
        return None

    def dispatched(self, goal, elapsed):
        if not goal.get('box_orbit') or self.last_choice is None:
            return
        item, sector = self.last_choice
        item['attempts'][sector] = item['attempts'].get(sector, 0) + 1
        if item['started'] is None:
            item['started'] = elapsed
        self.last_choice = None

    def summary(self):
        return [dict(center=[round(float(v), 2) for v in item['center']],
                     seen=len(item['bearings']), gap_deg=round(math.degrees(self.gap(item)[0])),
                     attempts=sum(item['attempts'].values())) for item in self.boxes]
