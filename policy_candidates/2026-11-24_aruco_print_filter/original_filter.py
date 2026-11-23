"""Check an ArUco Original outline for credible print structure without reading its ID."""
import cv2
import numpy as np

from osp_mission.hybrid_passing import WORDS


SIZE = 56
SQUARE = np.float32([[0, 0], [SIZE - 1, 0], [SIZE - 1, SIZE - 1], [0, SIZE - 1]])
CENTERS = (np.arange(7, dtype=np.float32) + .5) * (SIZE - 1) / 7
RIM = np.ones((7, 7), dtype=bool)
RIM[1:-1, 1:-1] = False


def looks_like_original(gray, quad):
    """Return whether ordered convex corners enclose a credible Original print."""
    points = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    if not np.isfinite(points).all() or not cv2.isContourConvex(points):
        return False
    if cv2.contourArea(points, oriented=True) < 0:
        points = points[::-1].copy()

    transform = cv2.getPerspectiveTransform(points, SQUARE)
    patch = cv2.warpPerspective(gray, transform, (SIZE, SIZE))
    cells = np.mean(patch.reshape(7, 8, 7, 8)[:, 2:6, :, 2:6], axis=(1, 3))
    dark = float(np.median(cells[RIM]))
    light = float(np.percentile(cells[~RIM], 85))
    contrast = light - dark
    if contrast < 32:
        return False

    cutoff = (light + dark) / 2
    bits = cells > cutoff
    dark_sides = (np.count_nonzero(~bits[0]), np.count_nonzero(~bits[-1]),
                  np.count_nonzero(~bits[:, 0]), np.count_nonzero(~bits[:, -1]))
    if np.count_nonzero(bits & RIM) > 5 or sum(side >= 5 for side in dark_sides) < 3:
        return False

    # Project the cell centres back onto the native image. The same source
    # pixel cannot provide independent evidence for more than one payload cell.
    centers = np.stack(np.meshgrid(CENTERS[1:-1], CENTERS[1:-1]), axis=-1)
    projected = cv2.perspectiveTransform(centers.reshape(1, -1, 2), np.linalg.inv(transform))[0]
    native = np.rint(projected).astype(int)
    _, neighbors, counts = np.unique(native, axis=0, return_inverse=True, return_counts=True)
    visible = ((native[:, 0] >= 0) & (native[:, 0] < gray.shape[1])
               & (native[:, 1] >= 0) & (native[:, 1] < gray.shape[0]))
    confident = (visible & (counts[neighbors] == 1)).reshape(5, 5)
    confident &= np.abs(cells[1:-1, 1:-1] - cutoff) >= .14 * contrast
    if np.count_nonzero(confident) < 18:
        return False

    payload = bits[1:-1, 1:-1]
    for rotation in range(4):
        observed = np.rot90(payload, rotation)
        usable = np.rot90(confident, rotation)
        differences = (observed[:, None, :] != WORDS) & usable[:, None, :]
        row_errors = differences.sum(axis=2).min(axis=1)
        supported_rows = (usable.sum(axis=1) >= 4) & (row_errors == 0)
        if row_errors.sum() <= 2 and np.count_nonzero(supported_rows) >= 3:
            return True
    return False
