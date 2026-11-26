"""Infer opposite chair-side room ends from live SLAM geometry."""
import math

import cv2
import numpy as np

from osp_mission.auto_chair_strip import chair_returns


class EndWalls:
    def __init__(self, grid):
        self.ends = ()
        free = np.argwhere((grid.cells >= 0) & (grid.cells <= 25))
        if len(free)*grid.resolution**2 < 28.:
            return
        free = free[::max(1, len(free)//6000)]
        xy = self.world(grid, free)
        self.origin = xy.mean(axis=0)
        _, axes = np.linalg.eigh(np.cov((xy-self.origin).T))
        axis = axes[:, -1]
        # Long wall segments are less sensitive than free-space PCA to the
        # long rays that pass through glass during an unfinished map.
        lines = cv2.HoughLinesP(np.uint8(grid.cells >= 65)*255, 1, math.pi/180,
                               threshold=max(18, round(1.8/grid.resolution)),
                               minLineLength=2.8/grid.resolution, maxLineGap=.25/grid.resolution)
        if lines is not None:
            delta = lines[:, 0, 2:]-lines[:, 0, :2]
            weights = np.linalg.norm(delta, axis=1)**2
            angles = np.arctan2(delta[:, 1], delta[:, 0])+grid.origin[2]
            orthogonal = np.sum(weights*np.exp(4j*angles))
            if abs(orthogonal) >= .72*weights.sum():
                direction = np.angle(orthogonal)/4.
                axis = np.array([math.cos(direction), math.sin(direction)])
        cross = np.array([-axis[1], axis[0]])
        projections = (xy-self.origin) @ np.column_stack((axis, cross))
        lo, hi = np.percentile(projections, [15., 85.], axis=0)
        spans = hi-lo
        chair = np.argwhere(chair_returns(grid))
        if not len(chair):
            return
        points = self.world(grid, chair)-self.origin
        basis = (axis, cross)
        proposals = []
        for index in np.argsort(spans)[::-1]:
            if spans[index] < 5.:
                continue
            direction, transverse = basis[index], basis[1-index]
            length, across = points @ direction, points @ transverse
            ends = []
            for sign, edge in ((-1., lo[index]), (1., hi[index])):
                near = sign*(length-edge) >= -.35
                if np.count_nonzero(near) < 8 or np.ptp(across[near]) < .60:
                    continue
                ends.append((sign, edge))
            if ends:
                proposals.append((len(ends), spans[index], direction, ends))
        if proposals:
            _, _, self.axis, ends = max(proposals, key=lambda proposal: proposal[:2])
            # A side independently needs measured chair evidence. The opposite
            # side can qualify on a subsequent SLAM update.
            self.ends = tuple(ends)

    @staticmethod
    def world(grid, cells):
        row, col = (cells+.5).T*grid.resolution
        x, y, angle = grid.origin
        c, s = math.cos(angle), math.sin(angle)
        return np.column_stack((x+c*col-s*row, y+s*col+c*row))

    def contains(self, point):
        if not self.ends:
            return False
        position = (np.asarray(point)-self.origin) @ self.axis
        return any(sign*(position-edge) >= .45 for sign, edge in self.ends)

    def wall_surface(self, point, inward_yaw):
        if not self.ends:
            return False
        position = (np.asarray(point)-self.origin) @ self.axis
        alignment = np.array([math.cos(inward_yaw), math.sin(inward_yaw)]) @ self.axis
        return any(sign*(position-edge) >= 0. and sign*alignment <= -.57
                   for sign, edge in self.ends)
