import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import argparse
from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
from rclpy.utilities import remove_ros_args
import sys
import torch
from py_pubsub_msgs.msg import ClassCoordinates
import numpy as np
import math
from scipy.interpolate import interp1d

CAMERA_ANGLE = 190
IMAGE_WIDTH = 1344


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

    def centroid_distance(self, bb_height, classification):
        if classification == "person":
            r_min = self.pixels_to_distance(bb_height, self.MIN_HUMAN_HEIGHT)
            r_max = self.pixels_to_distance(bb_height, self.MAX_HUMAN_HEIGHT)
        else:
            r_min = 10e-6
            r_max = 5.5
        return (r_min + r_max) / 2


class YOLOv8Detector:
    def __init__(self):
        self.model = YOLO("yolov8s.pt")

    def compute(self, image):
        if image is not None:
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            return results


class YOLOv5Detector:
    def __init__(self, model_name="yolov5s"):
        # Load YOLOv5 model
        self.model = torch.hub.load("ultralytics/yolov5", model_name, pretrained=True)

    def compute(self, image):
        """Process a single image"""
        if image is not None:
            results = self.model(image)
            return results


class MinimalPublisher(Node):
    def __init__(self, camera_id):
        super().__init__("minimal_publisher")
        self.camera_id = camera_id

        self.topic_name_image = f"annotated_images_{camera_id}"
        self.publisher_annotated_image = self.create_publisher(
            Image, self.topic_name_image, 10
        )

        self.topic_name_information = f"information_{camera_id}"
        self.publisher_information = self.create_publisher(
            ClassCoordinates, self.topic_name_information, 10
        )

        self.subscription = self.create_subscription(
            Image, f"Cam{camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        self.detector = YOLOv8Detector()

    def toData(self, result, classes):
        bounding_box_height = result[3] - result[1]
        x_min, x_max = result[0], result[2]
        angle_min = (x_min - 0.5 * IMAGE_WIDTH) * (CAMERA_ANGLE / IMAGE_WIDTH)
        angle_max = (x_max - 0.5 * IMAGE_WIDTH) * (CAMERA_ANGLE / IMAGE_WIDTH)
        if self.camera_id == 4:
            angle_min = angle_min - 90
            angle_max = angle_max - 90
        elif self.camera_id == 3:
            angle_min = angle_min + 90
            angle_max = angle_max + 90
        elif self.camera_id == 2:
            angle_min = angle_min + 180
            angle_max = angle_max + 180

        class_id = int(result[-1])  # Get class ID
        class_name = classes[class_id]  # Get class name

        return bounding_box_height, angle_min % 360, angle_max % 360, class_name

    def listener_callback(self, image):
        self.get_logger().info(f"Image received from Camera {self.camera_id}")
        cv_image = cv2.cvtColor(
            self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"),
            cv2.COLOR_BGR2RGB,
        )

        results = self.detector.compute(cv_image)
        if results:
            # Annotated image part
            """if self.detector.type() == YOLOv8Detector:"""
            annotated_image = results[0].plot()
            """elif self.detector.type() == YOLOv5Detector:"""
            """annotated_image = results.render()[0]"""
            encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, "rgb8"
            )
            self.publisher_annotated_image.publish(encoded_annotated_image)
            new_results = results[0].boxes.data.cpu().tolist()
            class_names = results[0].names  # Assuming this is how you get class names
            distance_calculator = DistanceCalculator()
            for result in new_results:
                # Informations part
                informations = ClassCoordinates()
                informations.header.stamp = self.get_clock().now().to_msg()
                bounding_box_height, angle_min, angle_max, class_name = self.toData(
                    result, class_names
                )
                distance_centroid = distance_calculator.centroid_distance(
                    bb_height=bounding_box_height, classification=class_name
                )
                a = distance_calculator.compute_axis_size(
                    angle_min,
                    angle_max,
                    bounding_box_height,
                    class_name,
                    is_major_axis=True,
                )
                b = distance_calculator.compute_axis_size(
                    angle_min,
                    angle_max,
                    bounding_box_height,
                    class_name,
                    is_major_axis=False,
                )
                theta_moy = (angle_min + angle_max) / 2
                covariance_matrix = distance_calculator.covariance_matrix_from_ellipse(
                    a, b, theta_moy
                )
                informations.classification = class_name
                informations.angle = theta_moy
                flattened_covariance_matrix = [
                    element for row in covariance_matrix for element in row
                ]
                informations.covariance_matrix = flattened_covariance_matrix
                informations.centroid_distance = distance_centroid
                self.publisher_information.publish(informations)


def main(args=None):
    # Initialize ROS without passing args
    rclpy.init()

    # Create an argument parser for your script
    parser = argparse.ArgumentParser(description="ROS 2 YOLO Object Detection Node")

    # Add your custom argument
    parser.add_argument("--cam", type=str, default="1", help="Camera identifier")

    # Parse the command line arguments
    custom_args = parser.parse_args()

    # Create and spin your node
    minimal_publisher = MinimalPublisher(custom_args.cam)
    rclpy.spin(minimal_publisher)

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
