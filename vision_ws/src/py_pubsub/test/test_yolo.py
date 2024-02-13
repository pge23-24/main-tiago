import unittest
from unittest.mock import patch, MagicMock
from vision_ws.src.py_pubsub.py_pubsub.yolo import YOLOv8Detector, YOLOv5Detector


class TestYOLOv8Detector(unittest.TestCase):
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.YOLO")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.LOGGER")
    def test_yolov8_init_success(self, mock_logger, mock_yolo):
        """Test YOLOv8Detector initialization with successful model load."""
        detector = YOLOv8Detector()
        mock_yolo.assert_called_with("models/best.pt")
        mock_logger.error.assert_not_called()

    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.YOLO")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.LOGGER")
    def test_yolov8_init_failure(self, mock_logger, mock_yolo):
        """Test YOLOv8Detector initialization with failed model load."""
        mock_yolo.side_effect = Exception("Test exception")
        detector = YOLOv8Detector()
        mock_logger.error.assert_called()

    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.YOLO")
    def test_compute(self, mock_yolo):
        """Test computing detections on an image."""
        mock_model = MagicMock()
        mock_yolo.return_value = mock_model
        detector = YOLOv8Detector()
        mock_image = MagicMock()
        detector.compute(mock_image)
        mock_model.track.assert_called_with(
            mock_image,
            conf=0.3,
            persist=True,
            tracker="bytetrack.yaml",
            classes=[0, 80],
        )


class TestYOLOv5Detector(unittest.TestCase):
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.torch.hub.load")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.YOLO")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.LOGGER")
    def test_yolov5_init_success(self, mock_logger, mock_yolo, mock_torch_hub):
        """Test YOLOv5Detector initialization with successful model load."""
        mock_torch_hub.return_value = MagicMock()
        detector = YOLOv5Detector()
        mock_torch_hub.assert_called_with(
            "ultralytics/yolov5", "custom", path="models/yolv5l_custom.pt"
        )
        mock_logger.error.assert_not_called()

    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.torch.hub.load")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.YOLO")
    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.LOGGER")
    def test_yolov5_init_failure(self, mock_logger, mock_yolo, mock_torch_hub):
        """Test YOLOv5Detector initialization with failed model load and fallback."""
        mock_torch_hub.side_effect = Exception("Test exception")
        mock_yolo.side_effect = Exception("Fallback exception")
        detector = YOLOv5Detector()
        mock_logger.error.assert_called()
        self.assertIsNone(detector.model)

    @patch("vision_ws.src.py_pubsub.py_pubsub.yolo.torch.hub.load")
    def test_compute(self, mock_torch_hub):
        """Test computing detections on an image."""
        mock_model = MagicMock()
        mock_torch_hub.return_value = mock_model
        detector = YOLOv5Detector()
        mock_image = MagicMock()
        detector.compute(mock_image)
        mock_model.assert_called_with(mock_image)


if __name__ == "__main__":
    unittest.main()
