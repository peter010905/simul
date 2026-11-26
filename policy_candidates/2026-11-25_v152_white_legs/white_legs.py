"""Use white camera patches to inspect thin mapped obstacles selectively."""
import json
import math

import cv2
import numpy as np
from scipy import ndimage


class ThinObstacles:
    def __init__(self):
        self.grid = None

    def update(self, grid):
        if grid is self.grid:
            return
        self.grid = grid
        labels, count = ndimage.label(grid.cells >= 65, structure=np.ones((3, 3)))
        thin = np.zeros(count + 1, dtype=bool)
        for ident, region in enumerate(ndimage.find_objects(labels), 1):
            if region is None:
                continue
            dimensions = np.array([axis.stop - axis.start for axis in region]) * grid.resolution
            if dimensions.max() <= .48 and dimensions.min() <= .18:
                thin[ident] = True
        occupied = thin[labels]
        self.distance = (ndimage.distance_transform_edt(~occupied) * grid.resolution
                         if occupied.any() else np.full(labels.shape, np.inf))
        # Keep a white patch from attaching to a nearby leg when its actual
        # range hit is closer to a larger occupied component.
        other = (labels > 0) & ~occupied
        self.other_distance = (ndimage.distance_transform_edt(~other) * grid.resolution
                               if other.any() else np.full(labels.shape, np.inf))

    def near(self, point, radius=.12):
        cell = self.grid.index(*point)
        return (self.grid.inside(*cell) and self.distance[cell] <= radius
                and self.distance[cell] < self.other_distance[cell])


def white_patches(image):
    """Small, locally distinct bright patches in the decoded RGB luminance."""
    mask = cv2.inRange(image, 215, 255)
    _, _, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    proposals = []
    for (x, y, width, height, area), (u, v) in zip(stats[1:], centers[1:]):
        if min(width, height) < 2 or max(width, height) > 32 or area < 5:
            continue
        if x < 3 or y < 3 or x + width + 3 >= image.shape[1] or y + height + 3 >= image.shape[0]:
            continue
        roi = image[y-3:y+height+3, x-3:x+width+3]
        if np.count_nonzero(roi < 175) < .20 * roi.size:
            continue
        proposals.append((float(u), float(v), int(area)))
    return sorted(proposals, key=lambda row: -row[2])[:8]


def matched_range(angles, ranges, bearing, minimum, maximum, increment):
    error = np.abs(np.arctan2(np.sin(angles-bearing), np.cos(angles-bearing)))
    valid = (error <= max(.012, 1.25*abs(increment))) & np.isfinite(ranges)
    valid &= (ranges >= max(minimum, .7)) & (ranges <= min(maximum, 4.))
    if not valid.any():
        return None
    indices = np.flatnonzero(valid)
    nearest = indices[np.argmin(error[indices])]
    distance = ranges[nearest]
    if np.count_nonzero(np.abs(ranges[indices]-distance) <= .12) < 2:
        return None
    return float(distance)


def publish_white_patches(node, msg, image, k):
    from rclpy.time import Time
    from rclpy.duration import Duration
    from std_msgs.msg import String
    from osp_mission.marker_relook import sight_in_odom

    stamp = Time.from_msg(msg.header.stamp)
    scan = node.scan
    if scan is None:
        return
    scan_stamp = Time.from_msg(scan.header.stamp)
    if abs((scan_stamp-stamp).nanoseconds) > 250_000_000:
        return
    patches = white_patches(image)
    if not patches:
        return
    odom = node.p('odom_frame')
    transforms = node.tf
    camera = transforms.lookup_transform(odom, msg.header.frame_id, stamp).transform
    laser = transforms.lookup_transform_full(scan.header.frame_id, scan_stamp,
        msg.header.frame_id, stamp, odom, timeout=Duration()).transform
    laser_odom = transforms.lookup_transform(odom, scan.header.frame_id, scan_stamp).transform
    rays = np.array([[[u, v]] for u, v, _ in patches], dtype=np.float64)
    rays = cv2.undistortPoints(rays, np.asarray(k).reshape(3,3), np.asarray(node.info.d))
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
    for (_, _, area), ray in zip(patches, rays[:, 0]):
        origin, direction = sight_in_odom(laser, np.r_[ray, 1.])
        bearing = math.atan2(direction[1], direction[0])
        # Correct for the camera/LiDAR baseline using the measured ray depth.
        for nominal_range in (1., 2., 3.5):
            aim = origin[:2] + nominal_range*direction[:2]/max(.01, np.linalg.norm(direction[:2]))
            bearing = math.atan2(aim[1], aim[0])
            distance = matched_range(angles,ranges,bearing,scan.range_min,scan.range_max,scan.angle_increment)
            if distance is None:
                continue
            hit = distance*np.array([math.cos(bearing), math.sin(bearing)])
            rel = hit-origin[:2]
            ray_norm = np.linalg.norm(direction[:2])
            if ray_norm < .5 or np.dot(rel,direction[:2]) <= 0:
                continue
            miss = abs(rel[0]*direction[1]-rel[1]*direction[0])/ray_norm
            if miss > max(.035,.012*distance):
                continue
            # sight_in_odom transforms directions and returns the origin.
            base, vector = sight_in_odom(laser_odom,np.r_[hit,0.])
            point = base + distance*vector
            observer = camera.translation
            node.print_pub.publish(String(data=json.dumps(dict(stamp=stamp.nanoseconds/1e9,
                point=point[:2].tolist(), observer=[observer.x,observer.y], size=area, white_leg=True),
                allow_nan=False)))
            break
