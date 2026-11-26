import unittest

from marker_cooldown import exploration_gain


class MarkerCooldownTests(unittest.TestCase):
    def test_only_near_registered_marker_is_deweighted(self):
        markers = [(0., 0.)]
        self.assertEqual(exploration_gain((0., 0.), markers), .18)
        self.assertEqual(exploration_gain((1.5, 0.), markers), .18)
        self.assertGreater(exploration_gain((1.9, 0.), markers), .18)
        self.assertLess(exploration_gain((1.9, 0.), markers), 1.)
        self.assertEqual(exploration_gain((2., 0.), markers), 1.)
        self.assertEqual(exploration_gain((0., 0.), ()), 1.)


if __name__ == '__main__':
    unittest.main()
