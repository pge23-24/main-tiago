import unittest
import numpy as np
from vision_ws.src.py_pubsub.py_pubsub.distance_calculator import (
    DistanceCalculator,
)


class TestDistanceCalculator(unittest.TestCase):
    def setUp(self):
        self.calculator = DistanceCalculator()

    def test_degrees_to_radians(self):
        self.assertAlmostEqual(self.calculator.degrees_to_radians(0), 0)
        self.assertAlmostEqual(
            self.calculator.degrees_to_radians(180), 3.141592653589793
        )
        self.assertAlmostEqual(
            self.calculator.degrees_to_radians(90), 3.141592653589793 / 2
        )

    def test_pixels_to_distance(self):
        # Assuming known correct value for pixel_size=960
        self.assertAlmostEqual(self.calculator.pixels_to_distance(960), 0.3)
        # Test extrapolation
        self.assertTrue(self.calculator.pixels_to_distance(50) > 3.5)
        # Test interpolation
        self.assertTrue(0.3 < self.calculator.pixels_to_distance(500) < 3.5)

    def test_distance_polar(self):
        # Test with known values
        self.assertAlmostEqual(self.calculator.distance_polar(0, 0, 1, 90), 1)
        self.assertAlmostEqual(self.calculator.distance_polar(1, 0, 1, 90), 2**0.5)

    def test_compute_axis_size(self):
        # Specific tests for this method would depend on its implementation details
        # Example:
        self.assertGreater(
            self.calculator.compute_axis_size(0, 90, 960, "person", True), 0
        )

    def test_covariance_matrix_from_ellipse(self):
        # Known values
        covariance_matrix = self.calculator.covariance_matrix_from_ellipse(1, 1, 0)
        self.assertTrue((covariance_matrix == [[1, 0], [0, 1]]).all())

    def test_centroid_distance(self):
        # Assuming a classification that is not 'person'
        self.assertAlmostEqual(
            self.calculator.centroid_distance(960, "non-person"), (10e-6 + 5.5) / 2
        )

    def test_covariance_matrix_from_data(self):
        # Test with a known dataset
        data = [960.0, 0.0, 90.0, 45.0, "person"]
        matrix, distance, theta_moy, class_name = (
            self.calculator.covariance_matrix_from_data(data)
        )
        self.assertIsInstance(matrix, np.ndarray)
        self.assertIsInstance(distance, float)
        self.assertIsInstance(theta_moy, float)
        self.assertIsInstance(class_name, str)


if __name__ == "__main__":
    unittest.main()
