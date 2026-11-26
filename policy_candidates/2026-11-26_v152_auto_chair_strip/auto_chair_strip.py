"""Find rows of small, discrete SLAM returns without changing navigation."""
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def chair_returns(grid):
    occupied = grid.cells >= 65
    labels, _ = ndimage.label(occupied, structure=np.ones((3, 3)))
    components = []
    for ident, region in enumerate(ndimage.find_objects(labels), 1):
        if region is None or any(part.start == 0 or part.stop == limit
                                 for part, limit in zip(region, occupied.shape)):
            continue
        sizes = [(part.stop-part.start)*grid.resolution for part in region]
        if max(sizes) > .25 or min(sizes) > .18:
            continue
        cells = np.argwhere(labels[region] == ident)
        cells += [part.start for part in region]
        components.append((ident, cells * grid.resolution))
    result = np.zeros(occupied.shape, dtype=bool)
    if len(components) < 5:
        return result
    centers = np.array([cells.mean(axis=0) for _, cells in components])
    neighbors = [set() for _ in components]
    # Compare actual occupied cell centers, conservatively using the 0.378 m
    # base diameter as an upper bound on the gap.
    tree = cKDTree(centers)
    for a, b in tree.query_pairs(.75):
        gap = np.linalg.norm(components[a][1][:, None] - components[b][1], axis=-1).min()
        if gap <= .378:
            neighbors[a].add(b)
            neighbors[b].add(a)
    selected = set()
    for seed, neighborhood in enumerate(tree.query_ball_point(centers, 1.25)):
        if len(neighborhood) < 5:
            continue
        # A whole connected cluster can include a second furniture row. Find
        # short coherent pieces instead of fitting all branches as one strip.
        options = set(neighborhood)
        group = [seed]
        options.remove(seed)
        for member in group:
            additions = neighbors[member] & options
            group.extend(additions)
            options.difference_update(additions)
        if len(group) < 5:
            continue
        points = centers[group]
        relative = points-points.mean(axis=0)
        _, _, axes = np.linalg.svd(relative, full_matrices=False)
        along = relative @ axes[0]
        across = relative @ axes[1]
        if np.ptp(along) < .95 or np.ptp(across) > min(.42, .30*np.ptp(along)):
            continue
        # The graph already verifies short gaps along a connected route.
        selected.update(components[i][0] for i in group)
    if selected:
        result = np.isin(labels, list(selected))
    return result


class ChairCueFilter:
    def __init__(self, grid):
        self.grid = grid
        self.distances = None

    def includes(self, point):
        if self.distances is None:
            chair = chair_returns(self.grid)
            if not chair.any():
                self.distances = ()
                return False
            other = (self.grid.cells >= 65) & ~chair
            self.distances = (ndimage.distance_transform_edt(~chair)*self.grid.resolution,
                              ndimage.distance_transform_edt(~other)*self.grid.resolution if other.any()
                              else np.full(chair.shape, np.inf))
        cell = self.grid.index(*point)
        if not self.distances or not self.grid.inside(*cell):
            return False
        chair, other = self.distances
        return chair[cell] <= .10 and chair[cell] < other[cell]


def automatic_box_fit_cells(grid):
    chair = chair_returns(grid)
    if not chair.any():
        return grid.cells
    fitting = grid.cells.copy()
    fitting[chair] = -1
    return fitting
