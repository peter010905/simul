"""Run the installed Focus decoder on antialiased pinhole views of a 10 cm print."""
import argparse
import csv
import itertools
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from osp_mission.marker_pair import decode


def exposure(info, code, distance, tilt, phase, blur):
    # Project each corner, including its depth difference under yaw. Render just
    # the print neighbourhood at 8x before integrating down to native pixels.
    k = info['k']
    center = np.array((k[2], k[5])) + phase
    offsets = np.array([[-.05, -.05], [.05, -.05], [.05, .05], [-.05, .05]])
    yaw = math.radians(tilt)
    depth = distance - offsets[:, 0] * math.sin(yaw)
    corners = center + offsets * (k[0]*math.cos(yaw), k[4]) / depth[:, None]
    low = np.floor(corners.min(axis=0) - 6).astype(int)
    high = np.ceil(corners.max(axis=0) + 6).astype(int)
    size = tuple((high - low).tolist())
    source = np.float32([[-.5, -.5], [code.shape[1]-.5, -.5],
                         [code.shape[1]-.5, code.shape[0]-.5], [-.5, code.shape[0]-.5]])
    target = np.float32((corners - low + .5) * 8 - .5)
    patch = cv2.warpPerspective(code, cv2.getPerspectiveTransform(source, target),
                                tuple(v*8 for v in size), flags=cv2.INTER_NEAREST,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=218)
    frame = np.full((info['height'], info['width']), 218, np.uint8)
    frame[low[1]:high[1], low[0]:high[0]] = cv2.resize(patch, size, interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(frame, (0, 0), blur), center


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_metrics', type=Path)
    info = json.loads(parser.parse_args().camera_metrics.read_text())['camera_info']
    cv2.setNumThreads(1)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    render = (cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker')
              else cv2.aruco.drawMarker)
    writer = csv.writer(sys.stdout)
    writer.writerow(('distance_m', 'yaw_deg', 'blur_sigma_px', 'test_images', 'correct_id',
                     'anonymous_or_id', 'wrong_id'))
    for distance, tilt, blur in itertools.product((1.5, 2.5, 4., 6.), (0, 45, 60), (.35, .9)):
        correct = seen = wrong = 0
        for ident, phase in itertools.product((7, 103), ((0, 0), (.25, .5), (.5, .25), (.75, .75))):
            code = render(dictionary, ident, 140)
            code = np.where(code != 0, 218, 24).astype(np.uint8)
            frame, center = exposure(info, code, distance, tilt, phase, blur)
            small = []
            quads, ids = decode(frame, dictionary, small=small)
            detected = {int(i) for quad, i in zip(quads, [] if ids is None else ids.ravel())
                        if np.linalg.norm(quad.reshape(4, 2).mean(axis=0) - center) <= 4}
            correct += ident in detected
            seen += ident in detected or any(np.linalg.norm(np.array(hit[:2]) - center) <= 4 for hit in small)
            wrong += bool(detected - {ident})
        writer.writerow((distance, tilt, blur, 8, correct, seen, wrong))


if __name__ == '__main__':
    main()
