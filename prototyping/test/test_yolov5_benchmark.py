import unittest
import os
import numpy as np
import cv2
from prototyping.src.yolov5_benchmark import YOLOv5Detector
import shutil


class TestYOLOv5Detector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model_name = "yolov5n"  # Use a small model for testing
        cls.detector = YOLOv5Detector(cls.model_name)
        cls.test_dir = "test_data"
        os.makedirs(cls.test_dir, exist_ok=True)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.test_dir)

    def create_test_image(self, image_name="test_image.jpg", image_size=(100, 100, 3)):
        image_path = os.path.join(self.test_dir, image_name)
        test_image = np.random.randint(0, 256, image_size, dtype=np.uint8)
        cv2.imwrite(image_path, test_image)
        return image_path

    def test_init(self):
        self.assertIsNotNone(self.detector.model)

    def test_process_image(self):
        image_path = self.create_test_image("test_image.jpg")
        result = self.detector.process_image(image_path)
        self.assertIsNone(
            result
        )  # Test is far from perfect, but its meaning is that it should detect nothing in a random image
        os.remove(image_path)

    def test_process_images_from_folder_empty(self):
        empty_dir = os.path.join(self.test_dir, "empty_folder")
        os.makedirs(empty_dir)
        stats = self.detector.process_images_from_folder(empty_dir)
        self.assertEqual(stats, (np.nan, np.nan, [np.nan, np.nan]))
        os.rmdir(empty_dir)

    def test_process_images_from_folder_with_images(self):
        self.create_test_image("image1.jpg")
        self.create_test_image("image2.jpg")

        stats = self.detector.process_images_from_folder(self.test_dir)
        self.assertEqual(len(stats), 3)
        self.assertTrue(all(isinstance(stat, (float, np.ndarray)) for stat in stats))

    def test_timing_statistics(self):
        self.create_test_image("image1.jpg")
        self.create_test_image("image2.jpg")

        mean, median, percentiles = self.detector.process_images_from_folder(
            self.test_dir
        )
        self.assertIsInstance(mean, float)
        self.assertIsInstance(median, float)
        self.assertIsInstance(percentiles, np.ndarray)
        self.assertEqual(len(percentiles), 2)


if __name__ == "__main__":
    unittest.main()
