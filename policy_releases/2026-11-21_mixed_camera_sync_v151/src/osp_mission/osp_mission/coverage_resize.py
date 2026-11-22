"""Retain camera evidence when an expanding SLAM grid moves its origin."""

import numpy as np
from scipy.ndimage import binary_erosion


def remap(values, old_data, old_geometry, data, resolution, origin):
    """Reproject to the nearest cell with unchanged local occupancy and side.

    SLAM Toolbox bounds follow observed coordinates, so their translations are
    not necessarily whole cells. Nearest-cell reprojection introduces at most
    half a cell of quantization per axis. Reject exact ties and changed 3x3
    neighbourhoods; never merge observation bins or physical boundary sides.
    """
    result = np.zeros(values.shape[:-2] + data.shape, dtype=values.dtype)
    if old_data is None or old_geometry is None:
        return result
    _, old_resolution, old_origin = old_geometry
    if old_resolution != resolution:
        return result
    offset = (np.asarray(old_origin) - np.asarray(origin)) / resolution
    if not np.isfinite(offset).all() or np.any(np.abs(offset - np.rint(offset)) >= .5-1e-6):
        return result
    shift = np.rint(offset).astype(int)[::-1]  # world x,y to array row,column
    source_low = np.maximum(0, -shift)
    source_high = np.minimum(old_data.shape, np.asarray(data.shape) - shift)
    if np.any(source_high <= source_low):
        return result
    source = tuple(slice(int(a), int(b)) for a, b in zip(source_low, source_high))
    destination = tuple(slice(int(a), int(b)) for a, b in zip(source_low + shift, source_high + shift))
    def states(grid):
        return np.where(grid < 0, -1, np.where(grid > 50, 1, 0))
    stable = states(old_data[source]) == states(data[destination])
    stable = binary_erosion(stable, structure=np.ones((3, 3)), border_value=0)
    stable &= data[destination] > 50
    result[(Ellipsis,) + destination] = values[(Ellipsis,) + source] * stable
    return result
