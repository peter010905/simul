import cv2
import numpy as np

from osp_mission.landmark_identity import resolved_ids
from osp_mission.marker_pair import SeparateMarkers, decode


def test_adjacent_original_codes_have_independent_confirmations():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    image = np.full((80, 125), 255, np.uint8)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    image[16:58, 10:52] = draw(dictionary, 103, 42)
    image[16:58, 58:100] = np.rot90(draw(dictionary, 7, 42))
    corners, ids = decode(image, dictionary)
    assert sorted(ids.ravel()) == [7, 103]
    accepted = dict(zip(ids.ravel(), corners))
    histories = {i: [(t, [x, 0, 0]) for t in range(3)] for i, x in [(7, 1), (103, 1.09)]}
    views = SeparateMarkers()
    for stamp in [1, 1, 2]:
        views.observe(stamp, accepted)
    assert not resolved_ids(histories, separate=views)[0]
    views.observe(3, accepted)
    assert resolved_ids(histories, separate=views)[0] == {7, 103}
    overlap = SeparateMarkers()
    for stamp in range(5):
        overlap.observe(stamp, {7: corners[0], 103: corners[0]})
    assert not resolved_ids(histories, separate=overlap)[0]
