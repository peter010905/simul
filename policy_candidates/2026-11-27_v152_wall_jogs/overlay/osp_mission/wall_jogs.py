""".67: short wall steps (pillars, recesses) on the long walls become face-inspection tasks.

The measured room outline (the wall lap's floor contour, chair ribbon closed) is simplified to a
polygon. A wall step is a run of short edges (each 0.2-1.5 m) between two long edges (>= 1.5 m)
that are parallel and offset from each other by at most 1.2 m: the same wall continuing past a
pillar or recess. Every short edge in such a run is a face. Room corners (perpendicular long
edges) and chair bands (no parallel long edges) are not steps; callers also exclude chair bands.
"""
import math

import cv2
import numpy as np

from scipy import ndimage

from osp_mission.auto_chair_strip import chair_returns
from osp_mission.wall_lap import measured_perimeter

SIMPLIFY_M = .20
LONG_WALL_M = 1.5
STEP_FACE_M = (.20, 1.5)
MAX_RUN_EDGES = 4
PARALLEL_COS = .95
MAX_STEP_OFFSET_M = 1.2
CHAIR_CLEARANCE_M = .8         # a step face this close to chair returns belongs to a chair band


def wall_jog_faces(grid, pose, excluded=lambda point: False):
    """[(face midpoint (map), inward unit normal, half width)] for wall steps on long walls."""
    boundary = measured_perimeter(grid, pose)
    if boundary is None:
        return []
    points = boundary[0]
    polygon = cv2.approxPolyDP(points.reshape(-1, 1, 2).astype(np.int32),
                               SIMPLIFY_M / grid.resolution, True).reshape(-1, 2)
    if len(polygon) < 4:
        return []
    world = np.array([grid.world(row, col) for col, row in polygon], dtype=float)
    n = len(world)
    ends = np.roll(world, -1, axis=0)
    lengths = np.linalg.norm(ends - world, axis=1)
    free = (grid.cells >= 0) & (grid.cells <= 25)
    occupied = grid.cells >= 65
    chairs = chair_returns(grid)
    chair_distance = (ndimage.distance_transform_edt(~chairs) * grid.resolution
                      if chairs.any() else None)
    directions = (ends - world) / np.maximum(lengths, 1e-9)[:, None]
    # The room's long axis: length-weighted orientation (mod 180 deg) of the long edges. Only the
    # two long walls carry steps; the end walls (glass, chair bands) are across this axis.
    long_edges = lengths >= LONG_WALL_M
    if not long_edges.any():
        return []
    doubled = np.arctan2(directions[:, 1], directions[:, 0]) * 2
    weights = lengths * long_edges
    axis_angle = math.atan2(float((weights * np.sin(doubled)).sum()), float((weights * np.cos(doubled)).sum())) / 2
    axis = np.array([math.cos(axis_angle), math.sin(axis_angle)])
    along_axis = np.abs(directions @ axis) >= PARALLEL_COS
    step_edges = set()
    for i in range(n):
        if lengths[i] < LONG_WALL_M or not along_axis[i]:
            continue
        run = []
        for k in range(1, MAX_RUN_EDGES + 2):
            j = (i + k) % n
            if lengths[j] >= LONG_WALL_M:
                parallel = abs(float(directions[i] @ directions[j])) >= PARALLEL_COS
                offset = abs(float(np.cross(directions[i], world[j] - world[i])))
                if run and parallel and offset <= MAX_STEP_OFFSET_M:
                    step_edges.update(run)
                break
            if not STEP_FACE_M[0] <= lengths[j] <= STEP_FACE_M[1]:
                break
            run.append(j)
    faces = []
    for i in sorted(step_edges):
        a, b = world[i], ends[i]
        middle = (a + b) / 2
        tangent = (b - a) / lengths[i]
        normal = np.array([-tangent[1], tangent[0]])
        inside = grid.index(*(middle + .15 * normal))
        outside = grid.index(*(middle - .15 * normal))
        inside_free = grid.inside(*inside) and free[inside]
        outside_free = grid.inside(*outside) and free[outside]
        if outside_free and not inside_free:
            normal = -normal
        elif not inside_free:
            continue
        # A measured surface: occupied cells right behind the face.
        support = 0
        for s in np.linspace(-.4, .4, 5):
            cell = grid.index(*(middle + s * lengths[i] * tangent - .10 * normal))
            support += bool(grid.inside(*cell) and occupied[cell])
        if support < 2 or excluded(middle):
            continue
        cell = grid.index(*middle)
        if chair_distance is not None and grid.inside(*cell) and chair_distance[cell] < CHAIR_CLEARANCE_M:
            continue
        faces.append((middle, normal, float(lengths[i] / 2)))
    return faces
