"""Detect the chair clusters that line the glass short sides of the room from a live SLAM grid.

In the physical room the two short sides are glass and are blocked by chair clusters.
LiDAR sees only chair legs there: many small, disconnected returns spread over a 1-3 m
deep band that spans almost the full room width, at the two ends of the long axis.
Markers are never placed there, so the band is to be treated as one wall.

Signals (no ground truth, no prior map):
  1. Room axis: dominant orientation of long straight occupied segments (the long walls).
  2. Cross extent: the two long walls bound the room across the axis.
  3. Small returns: 8-connected occupied components no larger than a chair leg base.
  4. Coverage: per position along the axis, the fraction of the room width that has a
     small return within +-0.4 m. A chair band covers most of the width; a table column
     contributes a handful of legs (about 0.2 of the width).
  5. A band must start at an end of the room and be deep enough; it ends where the
     coverage stays low for more than a gap tolerance.
The result is a straight virtual wall per detected end (position along the axis).
"""
import math

import cv2
import numpy as np
from scipy import ndimage

LEG_EXTENT = 0.30      # largest bounding-box side of a chair-leg return (m)
WINDOW = 0.40          # half window along the axis for coverage (m)
CROSS_BIN = 0.40       # width bins across the room (m)
START_COVERAGE = 0.45  # band seed
KEEP_COVERAGE = 0.30   # band continuation (a table column with chairs reaches about 0.4 at most,
                       # but it is separated from the band by an empty aisle longer than GAP)
GAP = 0.45             # allowed low-coverage gap inside a band (m)
MIN_DEPTH = 0.60       # minimum band depth (m)
END_REACH = 1.0        # a band must begin within this distance of the room end (m)
ELONGATION = 1.5       # the mapped room must be this much longer than wide (the room is about 1.8):
                       # an unfinished map makes an unseen interior look like a room end
MIN_WIDTH = 6.0        # mapped width across the long walls (m)
STRONG_COVERAGE = 0.65 # before the room is mapped long enough, only bands this dense count
                       # (no recorded wrong detection exceeded 0.58)
MAX_DEPTH = 3.5        # deeper bands merge with interior furniture: rejected, never guessed
MARGIN = 0.20          # virtual wall placed this far inside the innermost band return (m)


def room_axis(occupied, resolution):
    # One dilation makes thin (one-cell) or slightly ragged wall traces continuous.
    occupied = cv2.dilate(np.uint8(occupied), np.ones((3, 3), np.uint8)) > 0
    lines = cv2.HoughLinesP(np.uint8(occupied) * 255, 1, math.pi / 180,
                            threshold=max(18, round(1.8 / resolution)),
                            minLineLength=2.8 / resolution, maxLineGap=.25 / resolution)
    if lines is None:
        return None, None
    delta = (lines[:, 0, 2:] - lines[:, 0, :2]).astype(float)
    weights = np.linalg.norm(delta, axis=1) ** 2
    angles = np.arctan2(delta[:, 1], delta[:, 0])
    orthogonal = np.sum(weights * np.exp(4j * angles))
    if abs(orthogonal) < .6 * weights.sum():
        return None, None
    base = np.angle(orthogonal) / 4.
    # Of the two orthogonal directions, the long axis carries more line length.
    along = [np.sum(weights * np.cos(angles - a) ** 2) for a in (base, base + math.pi / 2)]
    axis = base if along[0] >= along[1] else base + math.pi / 2
    # Canonical direction so the two ends keep their sign between SLAM updates.
    axis = (axis + math.pi / 2) % math.pi - math.pi / 2
    return axis, lines[:, 0, :].astype(float)


class ChairBand:
    """ends: list of dict(sign, wall_u, start_u, depth_m, coverage) in the room axis frame."""

    def __init__(self, cells, resolution, origin, gate=True):
        self.cells, self.res, self.origin = cells, float(resolution), origin
        self.ends = []
        self.axis = None
        occupied = cells >= 65
        free = (cells >= 0) & (cells <= 25)
        if free.sum() * self.res ** 2 < 25.:
            return
        labels, n = ndimage.label(occupied, np.ones((3, 3)))
        if n == 0:
            return
        boxes = ndimage.find_objects(labels)
        ext = np.array([[(s.stop - s.start) * self.res for s in b] for b in boxes])
        small_id = 1 + np.flatnonzero(ext.max(axis=1) <= LEG_EXTENT)
        large = occupied & ~np.isin(labels, small_id)
        theta, lines = room_axis(large, self.res)
        if theta is None:
            return
        # grid cell (row, col) -> metric map x, y; then into (u along, v across)
        self.axis = theta + origin[2]
        self.u_dir = np.array([math.cos(self.axis), math.sin(self.axis)])
        self.v_dir = np.array([-self.u_dir[1], self.u_dir[0]])

        def uv(mask):
            rc = np.argwhere(mask)
            xy = self.world(rc)
            return xy @ self.u_dir, xy @ self.v_dir

        fu, fv = uv(free)
        # Long walls bound the free space across the axis; glass ends do not bound it along.
        lu, lv = uv(large)
        v_lo, v_hi = np.percentile(fv, [1., 99.])
        inside = (lv > v_lo - .4) & (lv < v_hi + .4)
        if inside.sum() < 50:
            return
        su, sv = uv(occupied & np.isin(labels, small_id))
        keep = (sv > v_lo + .15) & (sv < v_hi - .15)
        su, sv = su[keep], sv[keep]
        # The room ends are the extent of the long straight wall segments along the axis.
        # Returns seen through the glass are not long wall lines next to the cross limits.
        ends_u = []
        for x0, y0, x1, y1 in lines:
            a = self.world(np.array([[y0, x0], [y1, x1]]) - .5)
            d = a[1] - a[0]
            if np.linalg.norm(d) < 2.0 or abs(d @ self.u_dir) < .97 * np.linalg.norm(d):
                continue
            v_mid = a.mean(axis=0) @ self.v_dir
            if min(abs(v_mid - v_lo), abs(v_mid - v_hi)) <= .8:
                ends_u.extend(a @ self.u_dir)
        if len(ends_u) < 2 or len(su) < 10:
            return
        u_lo, u_hi = min(ends_u), max(ends_u)
        self.v_range = (float(v_lo), float(v_hi))
        self.u_range = (float(u_lo), float(u_hi))
        if v_hi - v_lo < MIN_WIDTH:
            return
        self.elongated = u_hi - u_lo >= ELONGATION * (v_hi - v_lo)
        need = START_COVERAGE if (self.elongated or not gate) else STRONG_COVERAGE
        self.v_range = (float(v_lo), float(v_hi))
        self.u_range = (float(u_lo), float(u_hi))
        step = .10
        grid_u = np.arange(u_lo - .5, u_hi + .5, step)
        nbins = max(1, int((v_hi - v_lo - .3) / CROSS_BIN))
        vb = np.clip(((sv - v_lo - .15) / CROSS_BIN).astype(int), 0, nbins - 1)
        order = np.argsort(su)
        su, vb = su[order], vb[order]
        coverage = np.zeros(len(grid_u))
        for k, u in enumerate(grid_u):
            a, b = np.searchsorted(su, [u - WINDOW, u + WINDOW])
            coverage[k] = len(np.unique(vb[a:b])) / nbins
        self.profile = (grid_u, coverage)
        for sign in (-1, 1):
            idx = np.arange(len(grid_u)) if sign < 0 else np.arange(len(grid_u))[::-1]
            end = u_lo if sign < 0 else u_hi
            seed = next((k for k in idx if coverage[k] >= START_COVERAGE), None)
            if seed is None or abs(grid_u[seed] - end) > END_REACH:
                continue
            last, gap = seed, 0.
            for k in idx[list(idx).index(seed):]:
                if coverage[k] >= KEEP_COVERAGE:
                    last, gap = k, 0.
                else:
                    gap += step
                    if gap > GAP:
                        break
            span = sorted((grid_u[seed], grid_u[last]))
            depth = abs(grid_u[last] - end)
            mean = coverage[min(seed, last):max(seed, last) + 1].mean()
            if depth < MIN_DEPTH or depth > MAX_DEPTH or mean < need:
                continue
            # The coverage window reaches WINDOW past the innermost leg; step back to
            # MARGIN inside it so the chairs are beyond the wall.
            inner = grid_u[last] + (WINDOW - MARGIN) * sign
            self.ends.append(dict(sign=sign, wall_u=float(inner), start_u=float(end),
                                  depth_m=float(abs(inner - end)), coverage=float(mean),
                                  band=[float(span[0]), float(span[1])]))

    def world(self, rc):
        row, col = (rc + .5).T * self.res
        x, y, a = self.origin[:3]
        c, s = math.cos(a), math.sin(a)
        return np.column_stack((x + c * col - s * row, y + s * col + c * row))

    def contains(self, point, margin=0.):
        """True beyond a virtual chair wall (inside the chair band)."""
        if not self.ends:
            return False
        u = np.asarray(point) @ self.u_dir
        return any(e["sign"] * (u - e["wall_u"]) >= -margin for e in self.ends)

    def wall_mask(self):
        """Grid cells beyond each virtual wall, between the long walls."""
        mask = np.zeros(self.cells.shape, bool)
        if not self.ends:
            return mask
        rc = np.indices(self.cells.shape).reshape(2, -1).T
        xy = self.world(rc)
        u, v = xy @ self.u_dir, xy @ self.v_dir
        sel = (v > self.v_range[0] - .5) & (v < self.v_range[1] + .5)
        hit = np.zeros(len(u), bool)
        for e in self.ends:
            hit |= sel & (e["sign"] * (u - e["wall_u"]) >= 0)
        return hit.reshape(self.cells.shape)


class ChairBandTracker:
    """Commit a chair wall only after two consecutive consistent detections; keep it after.

    Walls are stored in the map frame as (point on the wall, unit normal pointing into the
    chair band), so small axis changes between SLAM updates do not move a committed wall.
    """

    def __init__(self, agree_m=0.5, agree_rad=math.radians(20)):
        self.agree_m, self.agree_rad = agree_m, agree_rad
        self.pending = {}
        self.walls = {}

    def update(self, cells, resolution, origin):
        band = ChairBand(cells, resolution, origin)
        seen = {}
        for e in band.ends:
            normal = e["sign"] * band.u_dir
            centre_v = (band.v_range[0] + band.v_range[1]) / 2
            point = e["wall_u"] * band.u_dir + centre_v * band.v_dir
            seen[e["sign"]] = (point, normal)
        for key, (point, normal) in seen.items():
            # Match by direction to an existing or pending wall (sign alone can flip with the axis).
            slot = next((k for k, (p, n) in list(self.walls.items()) + list(self.pending.items())
                         if n @ normal > math.cos(self.agree_rad)), None)
            if slot in self.walls:
                p, n = self.walls[slot]
                if abs((point - p) @ n) <= self.agree_m:
                    self.walls[slot] = ((p + point) / 2, n)
                continue
            if slot in self.pending:
                p, n = self.pending.pop(slot)
                if abs((point - p) @ n) <= self.agree_m:
                    self.walls[slot] = ((p + point) / 2, n)
                    continue
            self.pending[len(self.walls) + len(self.pending) + 10 * key + 100] = (point, normal)
        # A pending detection not repeated in the next update is dropped.
        for k in [k for k in self.pending if not any(
                self.pending[k][1] @ n > math.cos(self.agree_rad) for n in (v[1] for v in seen.values()))]:
            self.pending.pop(k)
        return self.walls

    def contains(self, point, margin=0.):
        q = np.asarray(point)
        return any((q - p) @ n >= -margin for p, n in self.walls.values())

    def project(self, point, standoff=1.0):
        """Move a point that lies within `standoff` of a committed wall (or beyond it)
        to `standoff` in front of that wall. Returns None when no wall applies."""
        q = np.asarray(point, float)
        moved = None
        for p, n in self.walls.values():
            depth = float((q - p) @ n)
            if depth > -standoff:
                q = q - n * (depth + standoff)
                moved = q
        return moved
