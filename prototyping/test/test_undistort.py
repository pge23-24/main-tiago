import unittest
import numpy as np
import cv2
from prototyping.src.surround_yolo.undistort import ImageUndistorter


class TestImageUndistorter(unittest.TestCase):
    def setUp(self):
        # Paramètres de caméra fictifs pour les tests
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
        self.dist_coeffs = np.array([0.1, -0.25, 0.001, 0.001, 0.0])
        self.undistorter = ImageUndistorter(self.camera_matrix, self.dist_coeffs)

        # Créer une image de test (image noire avec un carré blanc au centre)
        self.test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(self.test_image, (220, 140), (420, 340), (255, 255, 255), -1)

    def test_undistort_not_none(self):
        # Tester si la fonction undistort renvoie une image non nulle
        undistorted_image = self.undistorter.undistort(self.test_image)
        self.assertIsNotNone(undistorted_image)

    def test_undistort_valid(self):
        # Tester si l'image corrigée est valide
        undistorted_image = self.undistorter.undistort(self.test_image)
        self.assertIsNotNone(undistorted_image)
        self.assertGreater(undistorted_image.shape[0], 0)
        self.assertGreater(undistorted_image.shape[1], 0)


if __name__ == "__main__":
    unittest.main()
