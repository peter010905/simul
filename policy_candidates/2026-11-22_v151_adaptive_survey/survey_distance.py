"""Four-connected path length on the already safety-filtered SLAM grid."""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


def path_distances(traversable, start, resolution):
    cells = np.arange(traversable.size, dtype=np.int32).reshape(traversable.shape)
    horizontal = traversable[:, :-1] & traversable[:, 1:]
    vertical = traversable[:-1, :] & traversable[1:, :]
    first = np.concatenate((cells[:, :-1][horizontal], cells[:-1, :][vertical]))
    second = np.concatenate((cells[:, 1:][horizontal], cells[1:, :][vertical]))
    graph = csr_matrix((np.ones(first.size), (first, second)),
                       shape=(cells.size, cells.size))
    steps = dijkstra(graph, directed=False, unweighted=True,
                     indices=int(cells[start])).reshape(cells.shape)
    finite = np.isfinite(steps)
    # Reproduce repeated metric addition, including ties in baseline rankings.
    metres = np.full(int(steps[finite].max()) + 1, resolution, dtype=float)
    metres[0] = 0.
    steps[finite] = np.cumsum(metres)[steps[finite].astype(int)]
    return steps
