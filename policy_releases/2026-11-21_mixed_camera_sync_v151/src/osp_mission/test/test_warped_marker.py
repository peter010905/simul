import cv2
import numpy as np

from osp_mission.warped_marker import decode_rejected


def test_nonuniform_print_and_missing_code():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    x = np.linspace(0, 1, 70)
    source_x = np.interp(x, x + .40*x*(1-x), x) * 69
    quad = np.float32([[[6, 6], [75, 6], [75, 75], [6, 75]]])
    for ident in (103, 7, 42):
        code = draw(dictionary, ident, 70)
        image = np.pad(code[:, np.rint(source_x).astype(int)], 6, constant_values=255)
        recovered = decode_rejected(image, quad)
        assert recovered is not None and recovered[1] == ident
    assert decode_rejected(np.full((82, 82), 100, np.uint8), quad) is None
