import math
import unittest
from robot_extras.geometry import range_clear, standoff_goal


class ExtrasTest(unittest.TestCase):
    def test_standoff_and_heading(self):
        x, y, yaw = standoff_goal((1., 2.), (1., 5.), 1.)
        self.assertAlmostEqual(x, 1.)
        self.assertAlmostEqual(y, 4.)
        self.assertAlmostEqual(yaw, math.pi/2)

    def test_inside_standoff_does_not_reverse(self):
        self.assertIsNone(standoff_goal((0., 0.), (.5, 0.)))

    def test_invalid_range(self):
        for target in [(math.nan, 0.), (6., 0.)]:
            with self.assertRaises(ValueError):
                standoff_goal((0., 0.), target)

    def test_single_range_semantics(self):
        self.assertFalse(range_clear(-math.inf, .05, 10., .4))
        self.assertFalse(range_clear(math.nan, .05, 10., .4))
        self.assertFalse(range_clear(math.inf, .05, 10., .4))
        self.assertTrue(range_clear(math.inf, .05, 10., .4, True))
        self.assertFalse(range_clear(.2, .05, 10., .4))
        self.assertTrue(range_clear(.8, .05, 10., .4))


if __name__ == '__main__':
    unittest.main()
