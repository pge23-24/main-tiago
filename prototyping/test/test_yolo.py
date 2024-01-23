import unittest
from src.yolo import yolo  
import cv2  # Assuming OpenCV is used for image handling
from PIL import Image
import numpy as np
class TestYolo(unittest.TestCase):

    def setUp(self):
        """ Set up the YOLO instance before each test """
        self.yolo_instance = yolo()

    def test_compute_with_valid_image(self):
        """ Test the compute function with a valid image """
        # Load a test image (replace 'path_to_test_image.jpg' with a valid image path)
        random_image = Image.fromarray(np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8))

        # Check if the image is not None
        self.assertIsNotNone(random_image, "Failed to load test image.")

        # Call the compute method
        results = self.yolo_instance.compute(random_image)

        # Assert that results are returned
        self.assertIsNotNone(results, "No results returned from compute method.")

    def test_compute_with_invalid_image(self):
        """ Test the compute function with an invalid image """
        invalid_image = None  # Simulate a failed image load

        # Call the compute method
        results = self.yolo_instance.compute(invalid_image)

        # Assert that no results are returned
        self.assertIsNone(results, "Results should not be returned for an invalid image.")

# This allows the test to be run from the command line
if __name__ == '__main__':
    unittest.main()