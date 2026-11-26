"""Map-only viewpoints in the observed free strip on the front side of a face."""
import math
import numpy as np
from scipy import ndimage


def clearance_grid(grid):
    free = (grid.cells >= 0) & (grid.cells <= 25)
    return ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1] * grid.resolution


def indices(grid, points):
    delta = points - np.asarray(grid.origin[:2])
    c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
    col = np.floor((c*delta[:, 0] + s*delta[:, 1])/grid.resolution).astype(int)
    row = np.floor((-s*delta[:, 0] + c*delta[:, 1])/grid.resolution).astype(int)
    return row, col


def inspection_pose(grid, cost, nav, distances, clearance, face, normal, half_width,
                    pose, goal_fields, veto, visibility_offset=.06):
    # Search the camera sector in metric coordinates, independently for each
    # face. Clearance to both boundaries shapes the valid interval at each offset.
    step = max(.05, grid.resolution)
    stand, shift = np.meshgrid(np.arange(.40, 1.501, step), np.arange(-.50, .501, step*2))
    stand, shift = stand.ravel(), shift.ravel()
    length = np.hypot(stand, shift)
    # Project both ends of the fitted face into a camera aimed at its centre.
    half_angle = np.arctan2(stand * half_width, length**2 - abs(shift) * half_width)
    valid = (length <= 1.5) & (abs(shift) <= .75*stand) & (half_angle <= math.radians(28))
    stand, shift = stand[valid], shift[valid]
    tangent = np.array([-normal[1], normal[0]])
    points = face + stand[:, None]*normal + shift[:, None]*tangent
    rows, cols = indices(grid, points)
    cr, cc = indices(cost, points)
    inside = ((rows >= 0) & (rows < grid.cells.shape[0]) & (cols >= 0) & (cols < grid.cells.shape[1]) &
              (cr >= 0) & (cr < nav.shape[0]) & (cc >= 0) & (cc < nav.shape[1]))
    ids = np.flatnonzero(inside)
    room = clearance[rows[ids], cols[ids]]
    walk = distances[rows[ids], cols[ids]]
    legal = (room >= .32) & np.isfinite(walk) & nav[cr[ids], cc[ids]]
    score = walk*.24 + 2.8*np.maximum(0., .70-room) + .6*abs(stand[ids]-.85) + .35*abs(shift[ids])
    for i in ids[legal][np.argsort(score[legal], kind='stable')]:
        point = points[i]
        heading = math.atan2(face[1]-point[1], face[0]-point[0])
        goal = dict(goal_fields, x=float(point[0]), y=float(point[1]),
                    yaw=heading-goal_fields['scan_angle']/2,
                    distance=float(np.linalg.norm(point-pose[:2])),
                    standoff_m=float(stand[i]), lateral_m=float(shift[i]),
                    clearance_m=float(clearance[rows[i],cols[i]]))
        if not veto(goal) and grid.line_clear(point, face + visibility_offset*normal):
            return float(distances[rows[i],cols[i]]), goal
    return None
