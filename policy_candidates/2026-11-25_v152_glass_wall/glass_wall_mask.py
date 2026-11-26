"""Ignore operator-marked chair/glass regions for inspection, not navigation."""
import json
import math
import os
from pathlib import Path


def inside_polygon(point, vertices):
    x, y = point
    contained = False
    for index, (ax, ay) in enumerate(vertices):
        bx, by = vertices[(index + 1) % len(vertices)]
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) < 1e-9 and min(ax, bx) <= x <= max(ax, bx) and min(ay, by) <= y <= max(ay, by):
            return True
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            contained = not contained
    return contained


class GlassWallMask:
    def __init__(self, path=None):
        run = os.environ.get('OSP_RUN_DIR')
        configured = os.environ.get('OSP_GLASS_WALL_FILE')
        self.path = Path(path or configured or (Path(run) / 'artifacts/glass_wall.json' if run else '/nonexistent/glass_wall.json'))
        self.segments = ()
        self.zones = ()
        self.signature = None
        self.skipped_groups = 0

    def refresh(self):
        try:
            stat = self.path.stat()
        except FileNotFoundError:
            self.segments, self.zones, self.signature = (), (), None
            return
        signature = stat.st_mtime_ns, stat.st_size
        if signature == self.signature:
            return
        try:
            data = json.loads(self.path.read_text())
            if data['frame'] != 'map' or len(data.get('segments', [])) > 8 or len(data.get('zones', [])) > 4:
                raise ValueError('Expected map-frame exclusion regions')
            segments = []
            for first, last in data.get('segments', []):
                points = tuple(tuple(float(value) for value in point) for point in (first, last))
                if any(len(point) != 2 or not all(map(math.isfinite, point)) for point in points):
                    raise ValueError('Invalid segment endpoint')
                if math.dist(*points) < .2:
                    raise ValueError('Glass segment is too short')
                segments.append(points)
            zones = []
            for raw in data.get('zones', []):
                if not 3 <= len(raw) <= 12:
                    raise ValueError('Region needs 3-12 vertices')
                vertices = tuple(tuple(float(value) for value in point) for point in raw)
                if any(len(point) != 2 or not all(map(math.isfinite, point)) for point in vertices):
                    raise ValueError('Invalid region vertex')
                twice_area = abs(sum(a[0] * b[1] - b[0] * a[1]
                                     for a, b in zip(vertices, vertices[1:] + vertices[:1])))
                if not .1 <= twice_area <= 80.:
                    raise ValueError('Region area is outside allowed bounds')
                zones.append(vertices)
        except (OSError, ValueError, TypeError, KeyError, IndexError):
            return
        self.segments, self.zones, self.signature = tuple(segments), tuple(zones), signature

    def includes(self, point, tolerance=.35):
        self.refresh()
        if self.in_zone(point):
            return True
        x, y = point
        for (ax, ay), (bx, by) in self.segments:
            dx, dy = bx - ax, by - ay
            fraction = max(0., min(1., ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
            if math.hypot(x - ax - fraction * dx, y - ay - fraction * dy) <= tolerance:
                return True
        return False

    def in_zone(self, point):
        self.refresh()
        return any(inside_polygon(point, vertices) for vertices in self.zones)

    def skip_generic(self, goal):
        """A marked region contains no inspection targets, including chair-like fitted faces."""
        if goal.get('r32_kind') != 'interior_surface':
            return False
        if self.in_zone(goal['surface_map']):
            return True
        return ('fitted_face' not in goal and goal.get('batch_box') is None
                and self.includes(goal['surface_map'], .8))

    def filter_samples(self, samples, grid):
        self.refresh()
        if not self.segments and not self.zones:
            self.skipped_groups = 0
            return samples
        kept = {}
        skipped = 0
        for key, cells in samples.items():
            point = grid.world(*cells[len(cells) // 2]) if cells else None
            if point is not None and (self.in_zone(point) or
                                      key[0] == 'wall' and self.includes(point)):
                skipped += 1
            else:
                kept[key] = cells
        self.skipped_groups = skipped
        return kept
