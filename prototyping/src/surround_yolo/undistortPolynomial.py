import cv2
import numpy as np


class UndistortPolynomial:

    def __init__(self, num_camera):
        # Camera parameters
        self.fp = 350  # Focal length (to define)
        if num_camera == 3:  # MAC address : 3c:69:d1:00:1e:37
            self.k1 = 351.707  # Polynomial distortion coefficient k1
            self.k2 = -36.2378  # Polynomial distortion coefficient k2
            self.k3 = 110.642  # Polynomial distortion coefficient k3
            self.k4 = -128.26  # Polynomial distortion coefficient k4
            self.k5 = 76.3589  # Polynomial distortion coefficient k5
            self.k6 = -18.3229  # Polynomial distortion coefficient k6
            self.Ox = 1.12514  # Offset point x
            self.Oy = -1.41038  # Offset point y
            self.aspect = 1.00037  # Aspect ratio
        elif num_camera == 1:  # MAC address : 3c:69:d1:00:1e:2a
            self.k1 = 350.239
            self.k2 = -17.2696
            self.k3 = 62.8516
            self.k4 = -72.9264
            self.k5 = 46.6648
            self.k6 = -12.2389
            self.Ox = -6.74071
            self.Oy = -8.05803
            self.aspect = 0.997769
        elif num_camera == 2:  # MAC address : 3c:69:d1:00:1e:2d
            self.k1 = 350.67
            self.k2 = -21.8104
            self.k3 = 75.5397
            self.k4 = -87.4609
            self.k5 = 54.1823
            self.k6 = -13.7281
            self.Ox = -5.55253
            self.Oy = 1.74807
            self.aspect = 0.998924
        elif num_camera == 4:  # MAC address : 3c:69:d1:00:1e:3f
            self.k1 = 350.324
            self.k2 = -21.0391
            self.k3 = 74.5099
            self.k4 = -89.1511
            self.k5 = 56.6033
            self.k6 = -14.4841
            self.Ox = -3.55022
            self.Oy = 3.77132
            self.aspect = 0.999075
        else:
            print(f"Le numero {num_camera} ne correspond à aucune camera du système."
                  f"Les paramètres caméra de la classe ont été mis pour la caméra 1")
            self.k1 = 350.239
            self.k2 = -17.2696
            self.k3 = 62.8516
            self.k4 = -72.9264
            self.k5 = 46.6648
            self.k6 = -12.2389
            self.Ox = -6.74071
            self.Oy = -8.05803
            self.aspect = 0.997769

    def undistort_image(self, image, cylindricalCorrection=True):
        # Convert image to grayscale (to get image size)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Get image size
        h, w = gray.shape[:2]

        # Generate normalized grid of pixel coordinates
        xn, yn = np.meshgrid(np.arange(w), np.arange(h))
        xn = xn.astype(np.float32)
        yn = yn.astype(np.float32)
        Pz = -1

        # Express coordinate from the center of the image
        xn = xn - (w / 2)
        yn = yn - (h / 2)

        # Convert pixel coordinates to normalized coordinates
        Px = xn * -Pz / self.fp
        Py = yn * -Pz / self.fp

        # Apply (or not) cylindrical correction
        if cylindricalCorrection:
            Pz = -np.cos(Px)
            Px = np.sin(Px)

        # Compute the incident angle
        theta = np.arccos(-Pz / np.sqrt(Px ** 2 + Py ** 2 + Pz ** 2))
        # theta = np.arctan2(np.sqrt(Px**2 + Py**2), -Pz)
        theta2 = theta * theta
        theta3 = theta2 * theta

        # Apply distortion equation
        r = self.k1 * theta + self.k2 * theta2 + self.k3 * theta3 + self.k4 * theta2 * theta2 + self.k5 * theta2 * theta3 + self.k6 * theta3 * theta3

        # Compute corresponding coordinates into the distorted image
        PxPrim = w / 2 + self.Ox + r * Px / np.sqrt(Px ** 2 + Py ** 2)
        PyPrim = h / 2 - self.Oy + self.aspect * r * Py / np.sqrt(Px ** 2 + Py ** 2)

        # Interpolate to obtain the undistorted image
        undistorted = cv2.remap(image, PxPrim, PyPrim, interpolation=cv2.INTER_LINEAR)

        # fill optical center
        undistorted[int(h / 2), int(w / 2)] = image[int(h / 2), int(w / 2)]

        return undistorted
