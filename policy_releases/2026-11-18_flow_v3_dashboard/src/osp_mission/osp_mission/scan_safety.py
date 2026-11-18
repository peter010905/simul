"""Short reverse-motion clearance in the robot base frame."""
import math
import numpy as np


def rear_clear(scan, sensor_pose, retreat=.20, radius=.18, margin=.05):
    """Require observed rear rays and an empty swept rear footprint.

    The scan mounting angle is not necessarily the robot heading. A missing
    return (+inf) is usable only when the sensor range exceeds the retreat.
    NaNs and below-minimum returns leave the corresponding direction unknown.
    Nav2 still performs its normal collision check during the BackUp action.
    """
    x, y, yaw = sensor_pose
    ranges = np.asarray(scan.ranges, dtype=float)
    if not len(ranges) or not np.isfinite([x, y, yaw, scan.angle_increment]).all():
        return False
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + yaw
    rear_angle = np.arctan2(np.sin(angles - math.pi), np.cos(angles - math.pi))
    rear = np.abs(rear_angle) < 1.3
    observed = np.sort(rear_angle[rear])
    if len(observed) < 9 or observed[0] > -1.15 or observed[-1] < 1.15:
        return False
    if np.max(np.diff(observed)) > .15:
        return False
    if scan.range_max <= retreat + radius + margin + math.hypot(x, y):
        return False
    if np.any(np.isnan(ranges[rear]) | (ranges[rear] <= scan.range_min)):
        return False
    valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    px = x + ranges[valid] * np.cos(angles[valid])
    py = y + ranges[valid] * np.sin(angles[valid])
    # The forward half of the current footprint is omitted: backing away
    # from a front contact must remain possible when the rear is clear.
    blocked = ((px < 0.) & (px > -(retreat + radius + margin))
               & (np.abs(py) < radius + margin))
    return not np.any(blocked)
