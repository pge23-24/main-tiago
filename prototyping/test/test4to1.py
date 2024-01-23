from prototyping.src.surround_yolo.image4to1 import CvFunction, Fuze
import unittest
from unittest.mock import patch, MagicMock
import cv2 as cv
import numpy as np
import os


class TestCvFunction(unittest.TestCase):
    def setUp(self):
        self.cv_function = CvFunction()

    def tearDown(self):
        # Remove the calibresult.png file if it exists
        if os.path.exists("calibresult.png"):
            os.remove("calibresult.png")

    @patch("cv2.imread")
    @patch("os.listdir")
    def test_calibrate(self, mock_listdir, mock_imread):
        # Setup mock responses
        mock_listdir.return_value = ["image1.jpg", "image2.jpg"]
        mock_imread.return_value = np.zeros(
            (480, 640, 3), dtype=np.uint8
        )  # Dummy image

        # Mock cv functions
        with patch(
            "cv2.findChessboardCorners", return_value=(True, np.zeros((42, 1, 2)))
        ), patch("cv2.cornerSubPix"), patch("cv2.drawChessboardCorners"), patch(
            "cv2.fisheye.calibrate", return_value=(True, np.eye(3), np.zeros(4), [], [])
        ):
            # Test the method
            folder_images = "test_folder"
            self.cv_function.calibrate(folder_images)

            # Assertions can include checks for calls to specific cv2 functions, file reading operations, etc.
            self.assertTrue(mock_listdir.called)
            self.assertTrue(mock_imread.called)

    @patch("cv2.getOptimalNewCameraMatrix", return_value=(np.eye(3), (0, 0, 640, 480)))
    @patch("cv2.undistort", return_value=np.zeros((480, 640, 3), dtype=np.uint8))
    def test_undistort(self, mock_undistort, mock_getOptimalNewCameraMatrix):
        # Dummy parameters
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        ret = True
        mtx = np.eye(3)
        dist = np.zeros(4)

        # Test the method
        self.cv_function.undistort(image, ret, mtx, dist)

        # Assertions
        mock_getOptimalNewCameraMatrix.assert_called_once()
        mock_undistort.assert_called_once()
        args, kwargs = mock_undistort.call_args
        self.assertTrue(
            isinstance(args[0], np.ndarray) and args[0].shape == image.shape
        )
        self.assertTrue(isinstance(args[1], np.ndarray) and args[1].shape == mtx.shape)
        self.assertTrue(isinstance(args[2], np.ndarray) and args[2].shape == dist.shape)


class TestFuze(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Create mock images with same height but different widths
        cls.img1 = np.zeros((100, 200, 3), dtype=np.uint8)
        cls.img2 = np.zeros((100, 150, 3), dtype=np.uint8)
        cls.img3 = np.zeros((100, 100, 3), dtype=np.uint8)
        cls.img4 = np.zeros((100, 250, 3), dtype=np.uint8)

        # Image with different height
        cls.img_diff_height = np.zeros((120, 200, 3), dtype=np.uint8)

    def test_concatenate_same_height(self):
        fuze = Fuze()
        result = fuze.concatener_images_horizontalement(
            [self.img1, self.img2, self.img3, self.img4]
        )
        self.assertEqual(result.shape[0], 100)
        self.assertEqual(result.shape[1], 200 + 150 + 100 + 250)  # Sum of widths

    def test_concatenate_different_heights(self):
        fuze = Fuze()
        with self.assertRaises(AssertionError):
            fuze.concatener_images_horizontalement(
                [self.img1, self.img_diff_height, self.img3, self.img4]
            )

    def test_concatenate_less_images(self):
        fuze = Fuze()
        with self.assertRaises(IndexError):
            fuze.concatener_images_horizontalement([self.img1, self.img2])

    def test_concatenate_empty_list(self):
        fuze = Fuze()
        with self.assertRaises(IndexError):
            fuze.concatener_images_horizontalement([])


if __name__ == "__main__":
    unittest.main()
