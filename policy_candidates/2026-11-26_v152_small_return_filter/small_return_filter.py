"""Keep tiny LiDAR returns out of object inspection and box fitting."""
import numpy as np
from scipy import ndimage


class SmallReturns:
    def __init__(self):
        self.grid = None

    def update(self, grid):
        if grid is self.grid:
            return
        self.grid = grid
        occupied = grid.cells >= 65
        labels, _ = ndimage.label(occupied, structure=np.ones((3, 3)))
        counts = np.bincount(labels.ravel())
        selected = np.zeros(len(counts), dtype=bool)
        for ident, region in enumerate(ndimage.find_objects(labels), 1):
            if region is None or any(axis.start == 0 or axis.stop == size
                                     for axis, size in zip(region, occupied.shape)):
                continue
            longest = max(axis.stop-axis.start for axis in region)*grid.resolution
            area = counts[ident]*grid.resolution**2
            selected[ident] = longest <= .25 and area <= .035
        self.cells = selected[labels]
        self.distances = None

    def matches(self, point):
        if not self.cells.any():
            return False
        cell = self.grid.index(*point)
        if not self.grid.inside(*cell):
            return False
        if self.distances is None:
            occupied = (self.grid.cells >= 65) & ~self.cells
            self.distances = (
                ndimage.distance_transform_edt(~self.cells)*self.grid.resolution,
                ndimage.distance_transform_edt(~occupied)*self.grid.resolution
                if occupied.any() else np.full(self.cells.shape, np.inf),
            )
        small, other = self.distances
        return small[cell] <= .12 and small[cell] < other[cell]


def exclude_small_from_fit(grid, fit_cells):
    returns = SmallReturns()
    returns.update(grid)
    if not returns.cells.any():
        return fit_cells
    result = fit_cells.copy() if fit_cells is grid.cells else fit_cells
    result[returns.cells] = -1
    return result
