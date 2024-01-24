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
from py_pubsub.msg import ClassCoordinates


class YOLOv8Detector:
    def __init__(self):
        # Load YOLOv8 model
        self.model = YOLO("yolov8s.pt")

    def compute(self, image):
        """Process a single image"""
        if image is not None:
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            return results


# TODO: test this class
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
        x_box = (result[0] + result[2]) / 2
        angle = (x_box - 0.5 * 1344) * (190 / 1344)
        if self.camera_id == 4:
            angle = angle - 90
        elif self.camera_id == 3:
            angle = angle + 90
        elif self.camera_id == 2:
            angle = angle + 180
        class_id = int(result[-1])  # Get class ID
        class_name = classes[class_id]  # Get class name
        return angle % 360, class_name

    def listener_callback(self, image):
        self.get_logger().info(f"Image received from Camera {self.camera_id}")
        cv_image = cv2.cvtColor(
            self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"),
            cv2.COLOR_BGR2RGB,
        )

        results = self.detector.compute(cv_image)
        if results:
            # Annotated image part
            annotated_image = results[0].plot()
            encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, "rgb8"
            )
            self.publisher_annotated_image.publish(encoded_annotated_image)
            new_results = results[0].boxes.data.cpu().tolist()
            class_names = results[0].names  # Assuming this is how you get class names
            for result in new_results:
                # Informations part
                informations = ClassCoordinates()
                informations.header.stamp = self.get_clock().now().to_msg()
                angle, class_name = self.toData(result, class_names)
                informations.classification = class_name
                informations.angle = angle
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
