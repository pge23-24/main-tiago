from prototyping.src.surround_yolo.image4to1_V3 import CvFunction, Fuze
import unittest
from unittest.mock import patch, mock_open
import numpy as np
import cv2 as cv
from PIL import Image, ImageDraw


class TestCvFunction(unittest.TestCase):
    def setUp(self):
        self.cv_function = CvFunction()
        self.mock_image = np.zeros((100, 100, 3), dtype=np.uint8)

    def mock_os_listdir(self, folder):
        # Mock for os.listdir, return a list of file names
        return ["image1.jpg", "image2.jpg"]

    def mock_cv_imread(self, file_path, width=1200, height=600, rows=9, cols=6):
        # Taille de chaque case
        block_width = width // cols
        block_height = height // rows

        # Création de l'image
        chessboard = Image.new("RGB", (width, height), color="white")
        draw = ImageDraw.Draw(chessboard)

        # Dessin de l'échiquier
        for row in range(rows):
            for col in range(cols):
                if (row + col) % 2 == 1:
                    top_left = (col * block_width, row * block_height)
                    bottom_right = (
                        top_left[0] + block_width,
                        top_left[1] + block_height,
                    )
                    draw.rectangle([top_left, bottom_right], fill="black")

        return chessboard

    def mock_findChessboardCorners(self, image, pattern_size, flags):
        # Mock a successful detection with predetermined corners
        corners = np.random.rand(
            6 * 9, 1, 2
        )  # Random corners for the sake of the example
        return True, corners

    def test_calibrate(self):
        with patch("os.listdir", side_effect=self.mock_os_listdir), patch(
            "cv2.imread", side_effect=self.mock_cv_imread
        ), patch(
            "cv2.findChessboardCorners", side_effect=self.mock_findChessboardCorners
        ):
            K, D, DIM = self.cv_function.calibrate("dummy_folder")

            # Assertions
            self.assertIsInstance(K, np.ndarray)
            self.assertIsInstance(D, np.ndarray)
            self.assertIsInstance(DIM, tuple)

    def test_undistort(self):
        # Assuming K, D, DIM are correctly set (mock or use values from calibrate test)
        K = np.eye(3)
        D = np.zeros((4, 1))
        DIM = (100, 100)

        undistorted_img = self.cv_function.undistort(self.mock_image, K, D, DIM)

        # Assertions to check if undistorted_img is as expected
        self.assertIsInstance(undistorted_img, np.ndarray)
        self.assertEqual(undistorted_img.shape, self.mock_image.shape)

        # More specific tests can be added based on the expected behavior of the undistort function


if __name__ == "__main__":
    unittest.main()
