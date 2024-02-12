import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
import torch
import time

from py_pubsub_msgs.msg import CameraDetectionStamped
from py_pubsub_msgs.msg import CameraDetectionStampedArray
from py_pubsub.distance_calculator import DistanceCalculator

CAMERA_ANGLE = 190
IMAGE_WIDTH = 1344

models_path = "models/"


class YOLOv8Detector:
    type = "YOLOv8"

    def __init__(self):
        try:
            self.model = YOLO(f"{models_path}v8_custom_chariot.pt")
        except Exception as e:
            LOGGER.error(f"Error loading default YOLOv8 model. {e}")

    def compute(self, image):
        if image is not None:
            results = self.model.track(
                image, conf=0.0, persist=True, tracker="bytetrack.yaml"
            )
            return results


class YOLOv5Detector:
    type = "YOLOv5"

    def __init__(self, model_name=f"{models_path}/yolv5l_custom.pt"):
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
    def __init__(self):
        super().__init__("minimal_publisher")
        self.declare_parameter("cam_id", 1)
        self.declare_parameter("yolo_version", 8)

        self.camera_id = (
            self.get_parameter("cam_id").get_parameter_value().integer_value
        )
        self.yolo_version = (
            self.get_parameter("yolo_version").get_parameter_value().integer_value
        )

        # Publishers
        self.topic_name_image = f"annotated_images_{ self.camera_id}"
        self.publisher_annotated_image = self.create_publisher(
            Image, self.topic_name_image, 10
        )

        self.topic_name_information = f"camera_detection_{ self.camera_id}"
        self.publisher_information = self.create_publisher(
            CameraDetectionStampedArray, self.topic_name_information, 10
        )

        # Subscribers
        self.subscription = self.create_subscription(
            Image, f"Cam{self.camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()

        if self.yolo_version == 8:
            self.detector = YOLOv8Detector()
        if self.yolo_version == 5:
            self.detector = YOLOv5Detector()

        self.get_logger().info(
            f"Camera { self.camera_id} Yolo {self.detector.type} Node has started"
        )

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
        start = time.time()
        self.get_logger().info(f"Image received from Camera {self.camera_id}")
        cv_image = cv2.cvtColor(
            self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"),
            cv2.COLOR_BGR2RGB,
        )

        results = self.detector.compute(cv_image)
        if results:
            # Annotated image part
            # check type of results
            # YOLOv8-v5
            if isinstance(results, list):
                annotated_image = results[0].plot()
                new_results = results[0].boxes.data.cpu().tolist()
                class_names = results[0].names

            # yoloV5
            else:
                annotated_image = results.render()[0]
                # For information datas
                new_results = results.xyxy[0].cpu().numpy().tolist()
                class_names = results.names

            encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, "rgb8"
            )

            self.publisher_annotated_image.publish(encoded_annotated_image)

            distance_calculator = DistanceCalculator()
            info_array = CameraDetectionStampedArray()
            info_array.header.stamp = self.get_clock().now().to_msg()
            for result in new_results:
                # Informations part
                informations = CameraDetectionStamped()
                informations.header = info_array.header

                covariance_matrix, distance_centroid, theta_moy, class_name = (
                    distance_calculator.covariance_matrix_from_data(
                        self.toData(result, class_names)
                    )
                )
                flattened_covariance_matrix = [
                    element for row in covariance_matrix for element in row
                ]

                informations.classification = class_name
                informations.coordinates = [distance_centroid, theta_moy]

                informations.covariance = flattened_covariance_matrix

                info_array.detections.append(informations)

            self.publisher_information.publish(info_array)
        end = time.time()
        t = end - start
        self.get_logger().info(f"Time : {t}")


def main(args=None):
    # Initialize ROS with args
    rclpy.init(args=args)

    # Create and spin your node
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
