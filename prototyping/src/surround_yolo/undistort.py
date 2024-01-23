import cv2


class ImageUndistorter:
    def __init__(self, camera_matrix, dist_coeffs):
        """
        Initialise la classe avec les paramètres intrinsèques de la caméra et les coefficients de distorsion.
        :param camera_matrix: Matrice de la caméra (numpy array de taille 3x3).
        :param dist_coeffs: Coefficients de distorsion (numpy array).
        """
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def undistort(self, image):
        """
        Corrige la distorsion de l'image fournie.
        :param image: Image à corriger (numpy array).
        :return: Image corrigée (numpy array).
        """
        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        undistorted_image = cv2.undistort(
            image, self.camera_matrix, self.dist_coeffs, None, new_camera_matrix
        )
        x, y, w, h = roi
        undistorted_image = undistorted_image[y : y + h, x : x + w]
        return undistorted_image
