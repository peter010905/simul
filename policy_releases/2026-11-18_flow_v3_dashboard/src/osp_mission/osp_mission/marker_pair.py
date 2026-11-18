"""Verify small Original tags and retain independently observed neighbours."""
import itertools
from osp_mission.warped_marker import decode_rejected

import cv2
import numpy as np


def printed_pattern_matches(image, corners, ident, dictionary):
    # Original tags have five payload cells plus a black border on each side.
    square = np.float32([[0, 0], [42, 0], [42, 42], [0, 42]])
    matrix = cv2.getPerspectiveTransform(corners.reshape(4, 2), square)
    patch = cv2.warpPerspective(image, matrix, (42, 42))
    cells = np.median(patch.reshape(7, 6, 7, 6)[:, 2:5, :, 2:5], axis=(1, 3))
    if not 0 <= ident < len(dictionary.bytesList):
        return False
    if hasattr(cv2.aruco, 'generateImageMarker'):
        code = cv2.aruco.generateImageMarker(dictionary, ident, 7)
    else:
        code = cv2.aruco.drawMarker(dictionary, ident, 7)
    white = code != 0
    dark, light = np.median(cells[~white]), np.median(cells[white])
    if light - dark < 28:
        return False
    # Uneven lighting may shift the cutoff. Keep a dark border and allow a
    # single ambiguous payload cell only if it lies near the intensity cutoff.
    for fraction in (.5, .4, .6):
        cutoff = dark + fraction * (light - dark)
        errors = (cells > cutoff) != white
        if not errors.any():
            return True
        if errors.sum() == 1 and errors[1:-1, 1:-1].any():
            if abs(cells[errors].item() - cutoff) < .2 * (light - dark):
                return True
    return False


def overlapping(a, b):
    area, _ = cv2.intersectConvexConvex(a.reshape(4, 2), b.reshape(4, 2))
    return area > 0


def decode(image, dictionary):
    results = []
    for scale in (1, 2):
        frame = image if scale == 1 else cv2.resize(
            image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary)
        if scale == 1:
            native_rejected = rejected
        if ids is None:
            continue
        for quad, ident in zip(corners, ids.ravel()):
            quad = quad / scale
            # Measure on native pixels even for a proposal from the larger image.
            edge = np.linalg.norm(quad[0] - np.roll(quad[0], 1, axis=0), axis=1).min()
            if edge < 10 or not printed_pattern_matches(image, quad, int(ident), dictionary):
                continue
            if scale == 2 and any(overlapping(quad, other) for other, _ in results):
                continue
            results.append((quad, int(ident)))
    for quad in native_rejected:
        if any(overlapping(quad, other) for other, _ in results):
            continue
        recovered = decode_rejected(image, quad)
        if recovered is not None:
            results.append(recovered)
    if not results:
        return [], None
    return [quad for quad, _ in results], np.array([[ident] for _, ident in results])


class SeparateMarkers:
    """Allow neighbouring map estimates only after three simultaneous clear views."""
    def __init__(self):
        self.last_stamp = -1
        self.counts = {}

    def observe(self, stamp, accepted):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        for (a, qa), (b, qb) in itertools.combinations(accepted.items(), 2):
            if overlapping(qa, qb):
                continue
            pair = tuple(sorted((a, b)))
            self.counts[pair] = min(3, self.counts.get(pair, 0) + 1)

    def established(self, a, b):
        return self.counts.get(tuple(sorted((a, b))), 0) >= 3
