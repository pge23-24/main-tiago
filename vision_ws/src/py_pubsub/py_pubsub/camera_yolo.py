import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
from rclpy.utilities import remove_ros_args
import sys


class YOLODetector:
    def __init__(self):
        self.model = YOLO("yolov8s-pose.pt")

    def compute(self, image):
        if image is not None:
            results = self.model.track(image, persist=True, tracker="bytetrack.yaml")
            return results


class MinimalPublisher(Node):
    def __init__(self, camera_id):
        super().__init__("minimal_publisher")
        self.camera_id = camera_id
        self.topic_name = f"annotated_images_{camera_id}"
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.subscription = self.create_subscription(
            Image, f"Cam{camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        self.detector = YOLODetector()

    def toData(self, result, keypoint=False):
        """Convert the object to JSON format."""
        if result.probs is not None:
            LOGGER.warning("Warning: Classify task do not support tojson yet.")
            return

        # Create list of detection dictionaries
        resultat = []
        data = result.boxes.data.cpu().tolist()
        h, w = (1, 1)
        for i, row in enumerate(data):  # xyxy, track_id if tracking, conf, class_id
            box = {
                "x1": row[0] / w,
                "y1": row[1] / h,
                "x2": row[2] / w,
                "y2": row[3] / h,
            }
            class_id = int(row[-1])
            name = result.names[class_id]
            results = {"name": name, "box": box}
            if result.boxes.is_track:
                results["track_id"] = int(row[-3])  # track ID
            if result.masks:
                # numpy array
                x, y = result.masks.xy[i][:, 0], result.masks.xy[i][:, 1]
                results["segments"] = {"x": (x / w).tolist(), "y": (y / h).tolist()}
            if keypoint:
                x, y, visible = (
                    result.keypoints[i].data[0].cpu().unbind(dim=1)
                )  # torch Tensor
                results["keypoints"] = {
                    "x": (x / w).tolist(),
                    "y": (y / h).tolist(),
                    "visible": visible.tolist(),
                }
            resultat.append(results)

        # Convert detections to JSON
        return resultat

    def listener_callback(self, image):
        self.get_logger().info(f"Image received from Camera {self.camera_id}")
        cv_image = cv2.cvtColor(
            self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"),
            cv2.COLOR_BGR2RGB,
        )

        results = self.detector.compute(cv_image)
        if results:
            annotated_image = results[0].plot()
            msg = self._cv_bridge.cv2_to_imgmsg(annotated_image, "rgb8")
            self.publisher.publish(msg)


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
