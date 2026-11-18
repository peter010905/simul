"""Image-only inspection requests for an as-yet unidentified printed square."""
import math

import cv2
import numpy as np


# Leave space outside the contour to sample the paper margin.
DEST = np.float32([[7, 7], [48, 7], [48, 48], [7, 48]])
Y, X = np.mgrid[:56, :56]
distance = np.maximum(abs(X-27.5), abs(Y-27.5))
RIM = (distance >= 23) & (distance <= 25)
BORDER = (distance >= 17.5) & (distance <= 19.5)
INNER = distance < 14


def paper_candidate(image, quad):
    points = quad.reshape(4, 2)
    edges = np.linalg.norm(points - np.roll(points, 1, axis=0), axis=1)
    size = edges.min()
    if size < 10 or size < .46*edges.max():
        return None
    if np.any(points < 4) or np.any(points > [image.shape[1]-5, image.shape[0]-5]):
        return None
    m = cv2.getPerspectiveTransform(points, DEST)
    patch = cv2.warpPerspective(image, m, (56, 56), borderMode=cv2.BORDER_REPLICATE)
    white, black = np.median(patch[RIM]), np.median(patch[BORDER])
    if white - black < 38:
        return None
    bits = patch > (white + black) / 2
    if bits[RIM].mean() < .78 or bits[BORDER].mean() > .15 or not .15 < bits[INNER].mean() < .82:
        return None
    # A plain window or sign can have a border, but lacks a two-dimensional code.
    samples = bits[13:43:4, 13:43:4]
    if np.count_nonzero(np.diff(samples, axis=0)) < 4 or np.count_nonzero(np.diff(samples, axis=1)) < 4:
        return None
    x, y = points.mean(axis=0)
    return float(x), float(y), float(size)


class VisualCue:
    def __init__(self):
        self.last_stamp = -math.inf
        self.track = None
        self.track_stamp = -math.inf
        self.samples = 0
        self.other_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.quiet_until = -math.inf

    def update(self, image, candidates, readable, stamp, focal, cx):
        if stamp <= self.last_stamp:
            return None
        self.last_stamp = stamp
        if readable:
            self.quiet_until = max(self.quiet_until, stamp + 2.)
        if stamp < self.quiet_until:
            self.track = None
            self.samples = 0
            return None
        matches = [p for quad in candidates if (p := paper_candidate(image, quad)) is not None]
        if not matches:
            return None
        nearby = []
        if self.track is not None and stamp - self.track_stamp <= .35:
            x, y, size = self.track
            nearby = [p for p in matches if math.dist(p[:2], (x,y)) <= max(6., .55*size)
                      and .65 <= p[2]/size <= 1.54]
        self.track = max(nearby or matches, key=lambda p:p[2])
        self.track_stamp = stamp
        self.samples = self.samples+1 if nearby else 1
        if self.samples < 2:
            return None
        x, y, size = self.track
        # Exclude a verified 4x4 at this candidate, even with other signs in view.
        corners, ids, _ = cv2.aruco.detectMarkers(image, self.other_dictionary)
        if ids is not None and any(cv2.pointPolygonTest(q, (x,y), False) >= 0 for q in corners):
            self.track = None
            self.samples = 0
            return None
        self.quiet_until = stamp + 12.
        return dict(stamp=stamp, angle=-math.atan2(x-cx, focal), uv=[x,y], min_edge_pixels=size)
