import unittest
from unittest.mock import patch, Mock
from src.yolo import yolo 

class TestYolo(unittest.TestCase):

    @patch('src.YOLO')  # Mock the YOLO class
    def setUp(self, mock_yolo):
        # Setup a mock for YOLO model
        self.mock_model = Mock()
        mock_yolo.return_value = self.mock_model

        # Instantiate the yolo class
        self.yolo_instance = yolo()

    def test_compute_with_valid_image(self):
        # Mock the model's track method
        self.mock_model.track.return_value = "mock_results"

        # Call the compute method with a mock image
        results = self.yolo_instance.compute("mock_image")

        # Assert that the track method was called correctly
        self.mock_model.track.assert_called_with("mock_image", persist=True, tracker="bytetrack.yaml")

        # Assert that the results are as expected
        self.assertEqual(results, "mock_results")

    def test_compute_with_none_image(self):
        # Test compute method with None as image
        results = self.yolo_instance.compute(None)

        # Assert that the track method was not called
        self.mock_model.track.assert_not_called()

        # Assert that the result is None or as expected for this case
        self.assertIsNone(results)

if __name__ == '__main__':
    unittest.main()