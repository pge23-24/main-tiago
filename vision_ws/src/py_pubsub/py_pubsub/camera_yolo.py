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


class YOLOv8Detector:
    def __init__(self):
        # Load YOLOv8 model
        self.model = YOLO("yolov8s.pt")

    def compute(self, image):
        """Process a single image"""
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
            String, self.topic_name_information, 10
        )

        self.subscription = self.create_subscription(
            Image, f"Cam{camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        self.detector = YOLOv8Detector()

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
            # Calculate center x-coordinate of the bounding box
            center_x = (row[0] + row[2]) / (2 * w)
            # Map the center x-coordinate to the angle range [-95°, 95°]
            angle = (center_x - 0.5) * 190 - 95

            box = {
                "x1": row[0] / w,
                "y1": row[1] / h,
                "x2": row[2] / w,
                "y2": row[3] / h,
                "angle": angle,
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
            # Annotated image part
            annotated_image = results[0].plot()
            encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                annotated_image, "rgb8"
            )
            self.publisher_annotated_image.publish(encoded_annotated_image)

            # Informations part
            informations = String()
            informations.data = str(
                self.toData(results)
            )  # TODO: Vérifier ici l'intégrité des données
            self.publisher_information.publish(informations)
            """self.publisher_information.publish(
                String(data=str(self.toData(results)))
            )"""


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
