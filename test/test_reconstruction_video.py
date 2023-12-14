import unittest
from unittest.mock import patch, MagicMock
import os
import cv2
from src.reconstruction_video import VideoReconstruction
import io
from contextlib import redirect_stdout


class TestVideoReconstruction(unittest.TestCase):
    def setUp(self):
        self.video_reconstruction = VideoReconstruction()

    @patch("os.listdir")
    @patch("cv2.VideoWriter")
    @patch("cv2.imread")
    def test_video_detection_with_valid_images(
        self, mock_imread, mock_videowriter, mock_listdir
    ):
        # Mocking listdir to return a list of files
        mock_listdir.return_value = ["image1.jpg", "image2.jpg"]

        # Mocking imread to return a valid image
        mock_image = MagicMock()
        mock_image.shape = (968, 1226, 3)
        mock_imread.return_value = mock_image

        # Mocking VideoWriter
        mock_videowriter.return_value = MagicMock()

        # Call the method
        self.video_reconstruction.video_detection()

        # Asserts
        self.assertEqual(mock_imread.call_count, 2)
        mock_videowriter.return_value.write.assert_called_with(mock_image)

    """@patch("os.listdir")
    def test_video_detection_with_empty_directory(self, mock_listdir):
        # Mocking listdir to return an empty list
        mock_listdir.return_value = []

        # Call the method
        with self.assertLogs("your_module.VideoReconstruction", level="INFO") as log:
            self.video_reconstruction.video_detection()

        # Check for log message
        self.assertIn("Error: Invalid image size or format", log.output[0])
"""  # TODO : Fix this test, the logger doesn't work. Don't know why. Must come back to it later.


if __name__ == "__main__":
    unittest.main()
