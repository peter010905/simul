"""Read a rejected square with modest non-uniform print foreshortening."""
from functools import lru_cache

import cv2
import numpy as np


@lru_cache(maxsize=1)
def original_patterns():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    patterns = {}
    for ident in range(len(dictionary.bytesList)):
        bits = draw(dictionary, ident, 7) > 0
        for turns in range(4):
            key = np.rot90(bits, turns).tobytes()
            patterns[key] = (ident, turns) if key not in patterns else None
    return patterns


CENTERS = (np.arange(7) + .5) / 7.
OFFSETS = (0., .20, -.20, .40, -.40, .55, -.55)
# Cover either image axis, with the other axis sampled normally.
GRIDS = np.array([(CENTERS + dx * CENTERS * (1-CENTERS),
                   CENTERS + dy * CENTERS * (1-CENTERS))
                  for dx, dy in [(s, 0.) for s in OFFSETS] + [(0., s) for s in OFFSETS[1:]]])
INDEX = np.rint(55 * GRIDS).astype(int)
SQUARE = np.float32([[0, 0], [55, 0], [55, 55], [0, 55]])


def decode_rejected(image, corners):
    quad = corners.reshape(4, 2)
    if np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1).min() < 18:
        return None
    if np.any(quad < 3) or np.any(quad > [image.shape[1]-4, image.shape[0]-4]):
        return None
    patch = cv2.warpPerspective(image, cv2.getPerspectiveTransform(quad, SQUARE), (56, 56))
    patch = cv2.GaussianBlur(patch, (3, 3), .65)
    cutoff, _ = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cells = patch[INDEX[:, 1, :, None], INDEX[:, 0, None, :]].astype(float)
    patterns = original_patterns()
    matches = set()
    for values in cells:
        bits = values > cutoff
        match = patterns.get(bits.tobytes())
        if match is None:
            continue
        dark, light = np.median(values[~bits]), np.median(values[bits])
        if light-dark < 40 or np.count_nonzero(np.abs(values-cutoff)<.08*(light-dark)) > 3:
            continue
        matches.add(match)
    if len(matches) != 1:
        return None
    ident, turns = matches.pop()
    return np.roll(corners, turns, axis=1), ident
