"""Recover degraded outlines using only measured image pixels, without an ID dictionary."""
import math

import cv2
import numpy as np
from visual_cue import DEST, RIM, BORDER, INNER, X, Y, paper_candidate


def edge_candidate(image, contour):
    h, w = image.shape
    x, y, width, height = cv2.boundingRect(contour)
    left, right = x < 4, x+width > w-4
    if left == right or y < 5 or y+height > h-5 or height < 14:
        return None
    if not .54 <= width/height <= 1.35:
        return None
    # Away from the crop use the measured complete outline, even if its margin is cut.
    hull = cv2.convexHull(contour)
    quad = cv2.approxPolyDP(hull, .035*cv2.arcLength(hull, True), True)
    cut = x == 0 or x+width == w
    if not cut and len(quad)==4:
        points = np.float32(quad.reshape(4,2))
    elif cut:
        side = max(width, height)
        x0 = x+width-side if left else x
        points = np.float32([[x0,y],[x0+side-1,y],[x0+side-1,y+height-1],[x0,y+height-1]])
    else:
        return None
    center = points.mean(axis=0)
    if not 0 <= center[0] < w:
        return None
    transform = cv2.getPerspectiveTransform(points, DEST)
    patch = cv2.warpPerspective(image,transform,(56,56))
    # Map sample locations back to the source. Exclude interpolation at the crop boundary.
    uv = cv2.perspectiveTransform(np.float32(np.dstack((X,Y))),np.linalg.inv(transform))
    valid = (uv[:,:,0]>=1)&(uv[:,:,0]<w-2)&(uv[:,:,1]>=1)&(uv[:,:,1]<h-2)
    rim, border, inner = RIM&valid, BORDER&valid, INNER&valid
    if min(rim.sum()/RIM.sum(), border.sum()/BORDER.sum(), inner.sum()/INNER.sum()) < .47:
        return None
    white, black = np.median(patch[rim]), np.median(patch[border])
    if white-black < 45:
        return None
    bits = patch > (white+black)/2
    if bits[rim].mean()<.78 or bits[border].mean()>.17 or not .10<bits[inner].mean()<.82:
        return None
    # Require measured contrast along three sides. Missing-side pixels carry no weight.
    sides = (X<12, X>43, Y<12, Y>43)
    good_sides = 0
    for side in sides:
        r,b = side&rim,side&border
        if r.sum()>=18 and b.sum()>=18 and np.median(patch[r])-np.median(patch[b])>=40:
            good_sides += 1
    if good_sides < 3:
        return None
    region = np.s_[13:43:4,13:43:4]
    b, v = bits[region], valid[region]
    for axis in (0,1):
        neighbours = v[:-1]&v[1:] if axis==0 else v[:,:-1]&v[:,1:]
        if np.count_nonzero(np.diff(b,axis=axis)&neighbours)<3:
            return None
    edges = np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)
    if edges.min()<10 or edges.min()<.46*edges.max():
        return None
    return float(center[0]),float(center[1]),float(min(edges))


def borders(image):
    _, global_mask = cv2.threshold(image,0,255,cv2.THRESH_BINARY_INV|cv2.THRESH_OTSU)
    local_mask = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV,15,5)
    original, recovered = [], []
    for mask in (global_mask,local_mask):
        for contour in cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]:
            area = cv2.contourArea(contour)
            if not 65<=area<=image.size*.4:
                continue
            perimeter = cv2.arcLength(contour,True)
            quad = cv2.approxPolyDP(contour,.035*perimeter,True)
            match = None
            if len(quad)==4 and cv2.isContourConvex(quad):
                match = paper_candidate(image,quad.astype(np.float32))
            if match:
                original.append(match)
                continue
            x,y,w,h = cv2.boundingRect(contour)
            if min(w,h)<10 or max(w,h)>2.1*min(w,h):
                continue
            if x<4 or x+w>image.shape[1]-4:
                match = edge_candidate(image,contour)
            elif len(quad)>4:
                hull = cv2.convexHull(contour)
                # A broken ring can occupy only a third of its convex outline.
                if area < .30*cv2.contourArea(hull):
                    continue
                outline = cv2.approxPolyDP(hull,.04*cv2.arcLength(hull,True),True)
                if len(outline)==4:
                    match = paper_candidate(image,outline.astype(np.float32))
            if match:
                recovered.append(match)
    unique = []
    for match in original+recovered:
        if all(math.dist(match[:2],other[:2])>3 for other in unique):
            unique.append(match)
    return unique


def plane_hit(u, size, samples, camera, cal):
    """Stabilize a local normal on a ribbed surface without joining discontinuous depths."""
    from square_face import associate
    hit = associate(u, size, samples, camera, cal)
    if hit is None:
        return None
    point, normal = hit
    nearby = [p for p,_,_ in samples if np.linalg.norm(p-point)<=.14]
    if len(nearby)<4:
        return hit
    cloud = np.stack(nearby)
    center = cloud.mean(axis=0)
    offsets = cloud-center
    _, axes = np.linalg.eigh(offsets.T@offsets)
    n, tangent = axes.T
    if np.ptp(offsets@tangent)<.10 or np.max(np.abs(offsets@n))>.022:
        return hit
    direction = camera[2]-math.atan((u-cal.k[2])/cal.k[0])
    ray = np.array([math.cos(direction),math.sin(direction)])
    if n@ray>0:
        n = -n
    if n@ray>-.40:
        return hit
    depth = (n@(center-camera[:2]))/(n@ray)
    projected = camera[:2]+depth*ray
    if depth<=0 or np.linalg.norm(projected-point)>.065:
        return hit
    return projected,n


def can_confirm(u, size, width):
    # A clipped first sighting can start a track. To act, its next center must
    # leave enough room for a full minimum side across the image.
    return min(u, width-1-u)>=size/2
