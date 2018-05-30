import unittest

from util_geo_coordinates_ros import CoordinateTransform


class CoordinateTransformTest(unittest.TestCase):
    def test(self):
        latInit = 49.01439
        lonInit = 8.41722
        cs = CoordinateTransform(latInit, lonInit)
        [x, y] = cs.ll2xy(latInit, lonInit)
        self.assertAlmostEqual(x, 457386.38238563854)
        self.assertAlmostEqual(y, 5429219.051147663)

if __name__ == '__main__':
    unittest.main()
