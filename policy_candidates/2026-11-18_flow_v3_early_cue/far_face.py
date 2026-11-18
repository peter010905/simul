"""Range an anonymous distant image cue against a contiguous lidar surface."""
import math
import numpy as np


def far_face(u, size, scan, laser, camera, cal):
    r = np.asarray(scan.ranges)
    good = np.isfinite(r) & (r > max(.25, scan.range_min)) & (r < scan.range_max)
    indices = np.flatnonzero(good)
    angles = laser[2] + scan.angle_min + indices * scan.angle_increment
    xy = np.column_stack((np.cos(angles), np.sin(angles))) * r[indices,None] + laser[:2]
    delta = xy-camera[:2]
    bearings = np.arctan2(delta[:,1],delta[:,0])-camera[2]
    pixels = cal.k[2]-cal.k[0]*np.tan(bearings)
    in_strip = (np.cos(bearings)>0) & (np.abs(pixels-u)<=max(7.,min(12.,size/2)))
    ids,points,us = indices[in_strip],xy[in_strip],pixels[in_strip]
    if len(ids)<4 or np.any(r[ids]<=2.6) or np.any(r[ids]>=8.):
        return None
    # An unknown gap or foreground edge makes the image-to-depth match ambiguous.
    if np.any(np.diff(ids)!=1) or np.any(np.linalg.norm(np.diff(points,axis=0),axis=1)>.14):
        return None
    if not us.min()<=u<=us.max():
        return None
    center = np.mean(points,axis=0)
    _,axes = np.linalg.eigh((points-center).T@(points-center))
    normal,tangent = axes.T
    if np.ptp((points-center)@tangent)<.05 or np.max(np.abs((points-center)@normal))>.025:
        return None
    az = camera[2]-math.atan((u-cal.k[2])/cal.k[0])
    ray = np.array([math.cos(az),math.sin(az)])
    if normal@ray>0:
        normal=-normal
    if normal@ray>-.40:
        return None
    distance = (center-camera[:2])@normal/(ray@normal)
    point = camera[:2]+distance*ray
    along = (points-point)@tangent
    if distance<=0 or not along.min()<=0<=along.max():
        return None
    return point,normal
