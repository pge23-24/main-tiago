import unittest
from prototyping.src.distance_calculator import DistanceCalculator


class TestDistanceCalculator(unittest.TestCase):
    def setUp(self):
        self.calculator = DistanceCalculator()

    def test_degrees_to_radians(self):
        self.assertAlmostEqual(self.calculator.degrees_to_radians(0), 0)
        self.assertAlmostEqual(
            self.calculator.degrees_to_radians(180), 3.14159, places=5
        )
        self.assertAlmostEqual(self.calculator.degrees_to_radians(90), 1.5708, places=4)

    def test_pixels_to_distance(self):
        # Test avec des valeurs qui devraient être dans la plage interpolée
        self.assertGreater(self.calculator.pixels_to_distance(500), 0)
        # Test avec une valeur hors plage
        self.assertNotEqual(self.calculator.pixels_to_distance(10000), 0)

    def test_distance_polar(self):
        self.assertAlmostEqual(
            self.calculator.distance_polar(1, 0, 1, 90), 1.4142, places=4
        )
        self.assertAlmostEqual(
            self.calculator.distance_polar(1, 45, 1, 45), 0, places=4
        )

    def test_compute_axis_size(self):
        major_axis = self.calculator.compute_axis_size(0, 180, 300, "person", True)
        minor_axis = self.calculator.compute_axis_size(0, 180, 300, "person", False)
        self.assertGreater(major_axis, 0)
        self.assertGreater(minor_axis, 0)
        self.assertNotEqual(major_axis, minor_axis)

    def test_covariance_matrix_from_ellipse(self):
        cov_matrix = self.calculator.covariance_matrix_from_ellipse(2, 1, 45)
        self.assertEqual(cov_matrix.shape, (2, 2))
        self.assertNotEqual(cov_matrix[0, 0], cov_matrix[1, 1])
        self.assertEqual(cov_matrix[0, 1], cov_matrix[1, 0])


if __name__ == "__main__":
    unittest.main()
