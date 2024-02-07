import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
import torch
from py_pubsub_msgs.msg import ClassCoordinates

from py_pubsub.distance_calculator import DistanceCalculator

CAMERA_ANGLE = 190
IMAGE_WIDTH = 1344

models_path = "models/"


class YOLOv8Detector:
    type = "YOLOv8"

    def __init__(self):
        try:
            self.model = YOLO(f"{models_path}yolov8s.pt")
        except Exception as e:
            LOGGER.error(f"Error loading default YOLOv8 model. {e}")

    def compute(self, image):
        if image is not None:
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            return results


class YOLOv5Detector:
    type = "YOLOv5"

    def __init__(self, model_name="{models_path}/yolov5l.pt"):
        # Load YOLOv5 model
        try:
            self.model = torch.hub.load(
                "ultralytics/yolov5",
                "custom",
                path=f"{models_path}yolv5l_custom.pt",
            )
        except Exception as e:
            LOGGER.error(
                f"Error loading custom YOLOv5 model. {e} \nLoading default model."
            )
            try:
                self.model = YOLO(model_name)
            except Exception as e:
                LOGGER.error(f"Error loading default YOLOv5 model. {e}")
                self.model = None

    def compute(self, image):
        """Process a single image"""
        if image is not None:
            results = self.model(image)
            return results


class MinimalPublisher(Node):
    def __init__(self, camera_id, yolo_version):
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
        if yolo_version == "v8":
            self.detector = YOLOv8Detector()
        elif yolo_version == "v5":
            self.detector = YOLOv5Detector()

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
            # check type of results
            if isinstance(results, list):
                annotated_image = results[0].plot()
            else:
                annotated_image = results.render()[0]

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

                covariance_matrix, distance_centroid, theta_moy, class_name = (
                    distance_calculator.covariance_matrix_from_data(
                        self.toData(result, class_names)
                    )
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
    parser.add_argument("--yolo", type=str, default="v5", help="Yolo version")

    # Parse the command line arguments
    custom_args = parser.parse_args()

    # Create and spin your node
    minimal_publisher = MinimalPublisher(custom_args.cam, custom_args.yolo)
    rclpy.spin(minimal_publisher)

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
