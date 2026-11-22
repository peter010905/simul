"""A short second look at a decoded print leaving the image. No range or ID credit."""
import math

import numpy as np


def edge_print(native, corners, width, height, readable_pixels):
    """Accept a unique native, pattern-checked quad while ordinary decoding continues."""
    if len(native) != 1 or len(corners) != 1 or width < 2 or height < 2:
        return None
    ident, quad, pixels = native[0]
    if not 10 <= pixels < readable_pixels or not np.isfinite(quad).all():
        return None
    points = quad.reshape(4, 2)
    if ((points < 4).any() or (points > (width-5, height-5)).any()):
        return None
    u, v = points.mean(axis=0)
    offset = 2 * u / (width-1) - 1
    if abs(offset) < .78:
        return None
    return int(ident), float(u), float(v), float(pixels)


def clearance_at(grid, base, radius=.32):
    """Require the whole stationary footprint margin to lie on known free cells."""
    info = grid.info
    resolution = info.resolution
    position = info.origin.position
    q = info.origin.orientation
    if not all(math.isfinite(v) for v in (resolution, *base, position.x, position.y, q.x, q.y, q.z, q.w)):
        return False
    if (resolution <= 0 or info.width * info.height != len(grid.data) or
            abs(q.x) + abs(q.y) > .001 or abs(q.z*q.z + q.w*q.w - 1) > .001):
        return False
    angle = 2 * math.atan2(q.z, q.w)
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = base[0]-position.x, base[1]-position.y
    x, y = (c*dx+s*dy)/resolution, (-s*dx+c*dy)/resolution
    r = radius/resolution
    x0, x1, y0, y1 = math.floor(x-r), math.floor(x+r), math.floor(y-r), math.floor(y+r)
    if x0 < 0 or y0 < 0 or x1 >= info.width or y1 >= info.height:
        return False
    # Distance to the physical cell rectangle, including its boundaries.
    for row in range(y0, y1+1):
        for col in range(x0, x1+1):
            nearest = math.hypot(max(col-x, x-col-1, 0.), max(row-y, y-row-1, 0.))
            if nearest <= r and grid.data[row*info.width+col] != 0:
                return False
    return True


def camera_turn(cue, clock, camera, half_fov):
    origin = cue['origin_odom']
    bearing = cue['yaw_odom']
    if (len(origin) != 2 or len(camera) != 3 or not 0 < half_fov < math.pi/2 or
            not all(math.isfinite(value) for value in (*origin, *camera, bearing, clock, cue['stamp']))):
        return None
    if not -.1 <= clock-cue['stamp'] <= .65 or math.dist(origin, camera[:2]) > .18:
        return None
    delta = math.remainder(bearing-camera[2], math.tau)
    if not .12 <= abs(delta) <= min(.75, half_fov+.10):
        return None
    return delta


def budget(history, spent, now, duration, remaining, cue):
    """One attempt per ID, two per mission, and shared time with existing inspections."""
    tried = [entry for entry in history if entry.get('source') == 'edge_print']
    limit = min(16., .03*duration)
    if (len(tried) >= 2 or remaining < 10. or limit-spent < 5. or
            (history and now-history[-1]['started'] < 30.)):
        return 0.
    for entry in tried:
        if entry['candidate_id'] == cue['id'] or (
                math.dist(entry['odom_camera'][:2], cue['origin_odom']) < .8 and
                abs(math.remainder(entry['odom_yaw']-cue['yaw_odom'], math.tau)) < math.radians(12)):
            return 0.
    return min(8., limit-spent, remaining-2.)
