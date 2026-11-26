"""The marked wall is skipped while an adjacent object remains selectable."""
import json
import tempfile
import unittest
from pathlib import Path

from glass_wall_mask import GlassWallMask


class Grid:
    @staticmethod
    def world(row, col):
        return float(col), float(row)


class GlassWallMaskTests(unittest.TestCase):
    def test_only_marked_wall_samples_are_removed(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'glass_wall.json'
            mask = GlassWallMask(path)
            samples = {('wall', 1): [(0, 1)], ('interior', 2): [(0, 1)],
                       ('wall', 3): [(5, 1)]}
            self.assertEqual(mask.filter_samples(samples, Grid()), samples)
            path.write_text(json.dumps({'frame': 'map', 'segments': [[[0., 0.], [2., 0.]]]}))
            filtered = mask.filter_samples(samples, Grid())
            self.assertNotIn(('wall', 1), filtered)
            self.assertIn(('interior', 2), filtered)
            self.assertIn(('wall', 3), filtered)
            self.assertEqual(mask.skipped_groups, 1)
            generic = {'r32_kind': 'interior_surface', 'surface_map': (1., .6)}
            self.assertTrue(mask.skip_generic(generic))
            self.assertFalse(mask.skip_generic(dict(generic, fitted_face=2)))
            self.assertFalse(mask.skip_generic(dict(generic, batch_box=(1., .4))))
            self.assertFalse(mask.skip_generic(dict(generic, surface_map=(1., 1.1))))

    def test_marked_zone_removes_all_inspection_targets_inside_but_not_nearby_box(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'glass_wall.json'
            mask = GlassWallMask(path)
            path.write_text(json.dumps({'frame': 'map', 'segments': [], 'zones': [
                [[0., 0.], [2., 0.], [2., 1.], [0., 1.]]]}))
            samples = {('wall', 1): [(0, 1)], ('interior', 2): [(0, 1)],
                       ('interior', 3): [(2, 1)]}
            filtered = mask.filter_samples(samples, Grid())
            self.assertNotIn(('wall', 1), filtered)
            self.assertNotIn(('interior', 2), filtered)
            self.assertIn(('interior', 3), filtered)
            self.assertTrue(mask.skip_generic({'r32_kind': 'interior_surface',
                                               'surface_map': (1., .5), 'fitted_face': 2}))
            self.assertFalse(mask.skip_generic({'r32_kind': 'interior_surface',
                                                'surface_map': (1., 1.1), 'fitted_face': 2}))
            self.assertFalse(mask.skip_generic({'r32_kind': 'marker_approach',
                                                'surface_map': (1., .5)}))

    def test_invalid_edit_keeps_last_valid_segment(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'glass_wall.json'
            mask = GlassWallMask(path)
            path.write_text(json.dumps({'frame': 'map', 'segments': [[[0., 0.], [2., 0.]]]}))
            self.assertTrue(mask.includes((1., .3)))
            path.write_text('{broken')
            self.assertTrue(mask.includes((1., .3)))
            self.assertFalse(mask.includes((1., 1.)))


if __name__ == '__main__':
    unittest.main()
