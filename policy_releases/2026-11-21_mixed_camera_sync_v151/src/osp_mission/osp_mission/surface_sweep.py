"""Extend one observation turn along the visible part of a marker-sized surface."""
import math

import numpy as np

from osp_mission.wall_route import WallLook, wrap


def endpoint(points, center, camera):
    """Take the far angular extent of the same continuous surface, in odometry."""
    if len(points) < 10 or np.linalg.norm(points[-1] - points[0]) < .22:
        return None
    c = np.asarray(center)
    i = int(np.linalg.norm(points - c, axis=1).argmin())
    # Restrict to a contiguous, locally aligned strip: never bridge a corner
    # into the next wall or span the empty space between two objects.
    lo, hi = max(0, i - 3), min(len(points) - 1, i + 3)
    tangent = points[hi] - points[lo]
    length = np.linalg.norm(tangent)
    if length < .025:
        return None
    normal = np.array([-tangent[1], tangent[0]]) / length
    offsets = points - points[i]
    coherent = (np.abs(offsets @ normal) <= .065) & (np.linalg.norm(offsets, axis=1) <= .85)
    cuts = np.flatnonzero(~coherent)
    lo = max((j + 1 for j in cuts if j < i), default=0)
    hi = min((j for j in cuts if j > i), default=len(points))
    strip = points[lo:hi]
    if len(strip) < 10 or np.linalg.norm(strip[-1] - strip[0]) < .22:
        return None
    direction = math.atan2(c[1]-camera[1], c[0]-camera[0])
    sign = math.copysign(1., wrap(direction-camera[2]))
    angles = np.arctan2(strip[:, 1]-camera[1], strip[:, 0]-camera[0]) - direction
    extent = sign * np.arctan2(np.sin(angles), np.cos(angles))
    index = int(extent.argmax())
    return tuple(strip[index]) if extent[index] >= math.radians(8) else None


def extend_turn(turn, tip, camera, fov):
    if tip is None:
        return turn
    to_tip = math.atan2(tip[1]-camera[1], tip[0]-camera[0])
    sign = math.copysign(1., turn)
    extent = sign * wrap(to_tip - camera[2] - turn)
    # Keep the original center inside the settled camera frame.
    extra = min(extent, math.radians(24), fov / 3, math.radians(130)-abs(turn))
    return turn + sign * extra if extra >= math.radians(8) else turn


class SurfaceSweep(WallLook):
    def select(self, scan, laser, camera, fov, now):
        self.tip = None
        return super().select(scan, laser, camera, fov, now)

    def choose_extent(self, segment, center, camera):
        self.tip = endpoint(segment, center, camera)

    def observation_angle(self, node, turn, camera):
        widened = extend_turn(turn, self.tip, camera, node.inspection_camera[1])
        if widened != turn:
            node.get_logger().info(
                f'Surface sweep: center {math.degrees(turn):.0f} deg, end {math.degrees(widened):.0f} deg')
        return widened
