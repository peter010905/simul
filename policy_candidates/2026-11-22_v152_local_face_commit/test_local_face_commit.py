"""A wall glance must not discard a reachable unfinished box batch."""
import unittest
import numpy as np

from osp_mission.box_batch import BoxBatch
from osp_mission._r32_navigation_core import Grid
from osp_mission.center_wall import surface_cells


class LocalFaceCommitTests(unittest.TestCase):
    def test_unfitted_obstacle_has_a_stable_region_center(self):
        cells = np.zeros((100, 100), dtype=np.int8)
        cells[48:53, 48:53] = 100
        grid = Grid(cells, .1, (0., 0., 0.))
        samples, centers = surface_cells(grid, (2., 2., 0.), return_centers=True)
        self.assertTrue(any(kind == 'interior' for kind, *_ in samples))
        center = next(centers[component] for kind, component, *_ in samples if kind == 'interior')
        self.assertTrue(np.allclose(center, [5.05, 5.05]))

    def test_generic_mapped_box_keeps_its_neighbouring_face(self):
        batch = BoxBatch()
        batch.update([], [], (.5, 0.))
        first = {'surface_odom': (.2, 0.), 'previous_attempts': 0,
                 'generic_object_center_odom': (0., 0.)}
        batch.rank((1., first))
        batch.dispatched(first)
        second = {'surface_odom': (0., .2), 'previous_attempts': 0,
                  'generic_object_center_odom': (.15, .05)}
        batch.rank((1.2, second))
        self.assertTrue(second['local_batch'])

    def test_wall_glance_preserves_active_box_but_region_exit_clears_it(self):
        batch = BoxBatch()
        batch.update([(0., 0.)], [], (.5, 0.))
        batch.dispatched({'batch_box': (0., 0.)})
        batch.dispatched({'survey_wall': [1, 2, 3]})
        goal = {'surface_odom': (.2, 0.), 'previous_attempts': 0}
        batch.rank((1., goal))
        self.assertTrue(goal['local_batch'])
        batch.dispatched({'survey_region': True})
        other = {'surface_odom': (.2, 0.), 'previous_attempts': 0}
        batch.rank((1., other))
        self.assertFalse(other['local_batch'])


if __name__ == '__main__':
    unittest.main()
