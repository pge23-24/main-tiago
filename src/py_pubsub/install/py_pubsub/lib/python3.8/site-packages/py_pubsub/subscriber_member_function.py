import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from ultralytics import YOLO  # YOLOv8 import
from rclpy.utilities import remove_ros_args
import sys


class YOLODetector:
    def __init__(self):
        self.model = YOLO("yolov8s-pose.pt")  # Initialize the YOLO model

    def compute(self, image):
        # Run YOLOv8 tracking on the image if it's valid
        if image is not None:
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            return results


class MinimalSubscriber(Node):
    def __init__(self, camera_id="1"):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            Image, f"Cam{camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        self.detector = YOLODetector()

    def listener_callback(self, image):
        self.get_logger().info("Image received")
        cv_image = cv2.cvtColor(
            self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"),
            cv2.COLOR_BGR2RGB,
        )

        results = self.detector.compute(cv_image)
        if results:
            annotated_image = results[0].plot()
            # Add code to display or process the annotated image


def main(args=None):
    # Separate ROS arguments from script arguments
    ros_args = remove_ros_args(sys.argv)

    rclpy.init(args=ros_args)

    parser = argparse.ArgumentParser(description="ROS 2 YOLO Object Detection Node")
    parser.add_argument("--cam", type=str, default="1", help="Camera identifier")

    # Use parse_known_args to avoid error with unrecognized arguments
    custom_args, unknown_args = parser.parse_known_args(args=remove_ros_args(sys.argv))

    minimal_subscriber = MinimalSubscriber(custom_args.cam)
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
