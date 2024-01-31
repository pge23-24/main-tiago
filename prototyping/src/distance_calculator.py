import numpy as np
import math
from scipy.interpolate import interp1d


class DistanceCalculator:
    # Constants representing the range of human heights in meters
    MIN_HUMAN_HEIGHT = 1.5
    MAX_HUMAN_HEIGHT = 2.0

    # Data points for interpolation
    PIXEL_VALUES = np.array([960, 540, 336, 224, 172, 140, 120, 100])
    REAL_SIZES = np.array([0.3, 0.6, 1, 1.5, 2, 2.5, 3, 3.5])

    def __init__(self):
        # Interpolation function for converting pixel size to distance
        self.interp_func = interp1d(
            self.PIXEL_VALUES,
            self.REAL_SIZES,
            kind="quadratic",
            fill_value="extrapolate",
        )

    @staticmethod
    def degrees_to_radians(degrees):
        """
        Convert degrees to radians.

        :param degrees: Angle in degrees
        :return: Angle in radians
        """
        return degrees * math.pi / 180

    def pixels_to_distance(self, pixel_size, object_real_size=1.0):
        """
        Convert pixel size to real world distance.

        :param pixel_size: Size of an object in pixels
        :param object_real_size: Real world size of the object (default is 1 meter)
        :return: Distance in meters
        """
        return self.interp_func(pixel_size) * object_real_size

    def distance_polar(self, r1, theta1_deg, r2, theta2_deg):
        """
        Calculate the Euclidean distance between two points in polar coordinates with angles in degrees.

        :param r1: Radius of the first point
        :param theta1_deg: Angle of the first point in degrees
        :param r2: Radius of the second point
        :param theta2_deg: Angle of the second point in degrees
        :return: Euclidean distance
        """
        theta1 = self.degrees_to_radians(theta1_deg)
        theta2 = self.degrees_to_radians(theta2_deg)
        x1, y1 = r1 * math.cos(theta1), r1 * math.sin(theta1)
        x2, y2 = r2 * math.cos(theta2), r2 * math.sin(theta2)
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def compute_axis_size(
        self,
        theta_min_deg,
        theta_max_deg,
        bb_height,
        classification,
        is_major_axis=True,
    ):
        """
        Compute the size of an axis (major or minor) of an ellipse with angles in degrees.

        :param theta_min_deg: Minimum angle of the bounding box in degrees
        :param theta_max_deg: Maximum angle of the bounding box in degrees
        :param bb_height: Height of the bounding box in pixels
        :param is_major_axis: Boolean flag to compute major axis if True, else minor axis
        :return: Size of the axis
        """
        if classification == "person":
            r_min = self.pixels_to_distance(bb_height, self.MIN_HUMAN_HEIGHT)
            r_max = self.pixels_to_distance(bb_height, self.MAX_HUMAN_HEIGHT)
        else:
            r_min = 10e-6
            r_max = 5.5
        if is_major_axis:
            theta_mean_deg = (theta_min_deg + theta_max_deg) / 2
            return self.distance_polar(r_min, theta_mean_deg, r_max, theta_mean_deg)
        else:
            r_mean = (r_min + r_max) / 2
            return self.distance_polar(r_mean, theta_min_deg, r_mean, theta_max_deg)

    @staticmethod
    def covariance_matrix_from_ellipse(a, b, theta_deg):
        """
        Calculate the covariance matrix from ellipse parameters with the angle in degrees.

        :param a: Semi-major axis length
        :param b: Semi-minor axis length
        :param theta_deg: Rotation angle of the ellipse in degrees
        :return: 2x2 Covariance matrix
        """
        theta = DistanceCalculator.degrees_to_radians(theta_deg)
        sigma_x2 = a**2 * np.cos(theta) ** 2 + b**2 * np.sin(theta) ** 2
        sigma_y2 = a**2 * np.sin(theta) ** 2 + b**2 * np.cos(theta) ** 2
        covariance = (a**2 - b**2) * np.sin(theta) * np.cos(theta)
        return np.array([[sigma_x2, covariance], [covariance, sigma_y2]])
