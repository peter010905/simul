"""Four-connected metric travel on the mission's already-cleared grid."""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


def travel_distances(traversable, row, col, resolution):
    height, width = traversable.shape
    size = height * width
    # Only orthogonally adjacent free cells have an edge, including row breaks.
    hr, hc = np.nonzero(traversable[:, :-1] & traversable[:, 1:])
    vr, vc = np.nonzero(traversable[:-1] & traversable[1:])
    horizontal = hr * width + hc
    vertical = vr * width + vc
    src = np.concatenate((horizontal, horizontal + 1, vertical, vertical + width))
    dst = np.concatenate((horizontal + 1, horizontal, vertical + width, vertical))
    graph = csr_matrix((np.ones(src.size, dtype=bool), (src, dst)), shape=(size, size))
    hops = dijkstra(graph, indices=row * width + col, unweighted=True)
    reached = np.isfinite(hops)
    # Preserve the parent's per-step additions, including tie breaking at
    # distances that are not exactly representable in binary floating point.
    cumulative = np.empty(int(hops[reached].max()) + 1)
    cumulative[0] = 0.
    cumulative[1:] = resolution
    np.cumsum(cumulative, out=cumulative)
    hops[reached] = cumulative[hops[reached].astype(np.intp)]
    return hops.reshape(traversable.shape)
