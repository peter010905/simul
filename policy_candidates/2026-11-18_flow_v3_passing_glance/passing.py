"""Bound a re-centering request for a passing print, independently of ID registration."""
import math
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / '2026-11-18_flow_v3_edge_glance'))
from glance import EdgeGlance


# The Original dictionary encodes each five-cell row with one of four words.
WORDS = np.array([[1,0,0,0,0], [1,0,1,1,1], [0,1,0,0,1], [0,1,1,1,0]], dtype=bool)
SQUARE = np.float32([[0,0], [41,0], [41,41], [0,41]])
RIM = np.ones((7,7), dtype=bool)
RIM[1:-1,1:-1] = False


def printed_cells(image, quad):
    patch = cv2.warpPerspective(image, cv2.getPerspectiveTransform(quad, SQUARE), (42,42))
    cells = np.mean(patch.reshape(7,6,7,6)[:,2:5,:,2:5], axis=(1,3))
    dark, light = np.median(cells[RIM]), np.percentile(cells[1:-1,1:-1], 85)
    if light-dark < 38:
        return False
    bits = cells > (light+dark)/2
    if np.count_nonzero(bits[RIM]) > 1:
        return False
    payload = bits[1:-1,1:-1]
    for rotation in range(4):
        errors = np.count_nonzero(np.rot90(payload, rotation)[:,None,:] != WORDS, axis=2)
        if errors.min(axis=1).sum() <= 1:
            return True
    return False


def printed_at(image, candidate):
    u,v,size = candidate
    # Search only near a measured outline. A severely clipped print waits for
    # the existing two-frame surface tracker instead of filling in missing cells.
    radius = math.ceil(1.5*size)
    left, top = max(0,int(u)-radius), max(0,int(v)-radius)
    roi = image[top:int(v)+radius+1, left:int(u)+radius+1]
    _, mask = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    for contour in cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(contour) < .55*size**2:
            continue
        quad = cv2.approxPolyDP(contour, .04*cv2.arcLength(contour,True), True)
        if len(quad)!=4 or not cv2.isContourConvex(quad):
            continue
        points = quad.reshape(4,2).astype(np.float32) + np.float32([left,top])
        if np.linalg.norm(points.mean(axis=0)-[u,v]) > max(3,size*.22):
            continue
        if np.any(points < 1) or np.any(points > [image.shape[1]-2,image.shape[0]-2]):
            continue
        # OpenCV contour orientation can run either way. Preserve chirality.
        if cv2.contourArea(points, oriented=True) < 0:
            points = points[::-1].copy()
        if printed_cells(image, points):
            return True
    return False


class PassingGlance:
    def __init__(self):
        self.gate = EdgeGlance()

    def update(self, now, stamp, camera, image, focal, cx, contours, confirmed):
        width = image.shape[1]
        # A multi-frame surface match can already be identified as a hold by
        # the old tracker even while the print is moving away from the center.
        if confirmed and confirmed['action']=='hold':
            angle = math.degrees(-math.atan((confirmed['uv'][0]-cx)/focal))
            confirmed = dict(confirmed, dwell_s=.60)
            if abs(angle) >= 2.0:
                confirmed.update(action='turn', yaw_delta_deg=max(-6.,min(6.,angle)))
        eligible = []
        if confirmed is None and now-self.gate.last >= 12 and stamp>self.gate.last_stamp:
            eligible = [c for c in contours if min(c[0],width-1-c[0])<=.2*width and printed_at(image,c)]
        return self.gate.update(now, stamp, camera, width, focal, cx, eligible, confirmed)
