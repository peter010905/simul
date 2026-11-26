"""Exclude annotated chair returns from the box fit's private grid."""
import numpy as np
from scipy import ndimage

from osp_mission.glass_wall_mask import GlassWallMask, inside_polygon


CHAIR_STRIPS = GlassWallMask()


def box_fit_cells(grid, mask=CHAIR_STRIPS):
    mask.refresh()
    if not mask.zones:
        return grid.cells
    occupied = grid.cells >= 65
    labels, _ = ndimage.label(occupied, structure=np.ones((3, 3)))
    filtered = None
    for ident, region in enumerate(ndimage.find_objects(labels), 1):
        if region is None:
            continue
        if any(part.start == 0 or part.stop == limit
               for part, limit in zip(region, occupied.shape)):
            continue
        dimensions = [(part.stop - part.start)*grid.resolution for part in region]
        if max(dimensions) > .48 or min(dimensions) > .18:
            continue
        component = labels[region] == ident
        rows, cols = np.where(component)
        points = [grid.world(row+region[0].start, col+region[1].start)
                  for row, col in zip(rows, cols)]
        if not any(all(inside_polygon(point, vertices) for point in points)
                   for vertices in mask.zones):
            continue
        if filtered is None:
            filtered = grid.cells.copy()
        # Use unknown in the private fit grid. It remains occupied in the
        # original grid, which TwoViews and the viewpoint planner still use.
        filtered[region][component] = -1
    return grid.cells if filtered is None else filtered
