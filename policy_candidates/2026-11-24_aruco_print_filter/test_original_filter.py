import cv2
import numpy as np
import pytest

from original_filter import looks_like_original


QUAD = np.float32([[14, 8], [38, 14], [37, 27], [12, 29]])
SOURCE = np.float32([[0, 0], [69, 0], [69, 69], [0, 69]])


def render(cells, quad=QUAD):
    patch = cv2.resize(cells, (70, 70), interpolation=cv2.INTER_NEAREST)
    transform = cv2.getPerspectiveTransform(SOURCE, quad)
    return cv2.warpPerspective(patch, transform, (60, 45), flags=cv2.INTER_LINEAR, borderValue=238)


@pytest.mark.parametrize('ident, rotation', [(0, 0), (7, 1), (42, 2), (103, 3)])
def test_real_original_layout_survives_perspective(ident, rotation):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    cells = np.rot90(draw(dictionary, ident, 7), rotation)
    assert looks_like_original(render(cells), QUAD)


def test_checkerboard_and_plain_frame_are_rejected():
    cells = np.zeros((7, 7), dtype=np.uint8)
    cells[1:-1, 1:-1] = 255 * (np.indices((5, 5)).sum(axis=0) % 2)
    assert not looks_like_original(render(cells), QUAD)
    cells[1:-1, 1:-1] = 255
    assert not looks_like_original(render(cells), QUAD)


def test_a_tiny_silhouette_cannot_duplicate_native_pixel_evidence():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    quad = np.float32([[16, 14], [29, 14], [29, 17], [16, 17]])
    assert not looks_like_original(render(draw(dictionary, 42, 7), quad), quad)
