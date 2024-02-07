import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from ultralytics import YOLO  # YOLOv8 import
from ultralytics.utils import LOGGER  # LOGGER import
import torch
import time
from deep_sort_realtime.deepsort_tracker import DeepSort

from py_pubsub_msgs.msg import ClassCoordinates
from py_pubsub_msgs.msg import ClassCoordinatesArray
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

    def __init__(self, model_name=f"{models_path}/yolov5l.pt"):
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
    def __init__(self, camera_id, yolo_version, tracker_enabled):
        super().__init__("minimal_publisher")
        self.camera_id = camera_id

        self.tracker_enabled = tracker_enabled

        self.topic_name_image = f"annotated_images_{camera_id}"
        self.publisher_annotated_image = self.create_publisher(
            Image, self.topic_name_image, 10
        )

        self.topic_name_information = f"information_{camera_id}"
        self.publisher_information = self.create_publisher(
            ClassCoordinatesArray, self.topic_name_information, 10
        )

        self.subscription = self.create_subscription(
            Image, f"Cam{camera_id}/image_raw", self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        if yolo_version == "v8":
            self.detector = YOLOv8Detector()
        elif yolo_version == "v5":
            self.detector = YOLOv5Detector()

        if tracker_enabled:
            self.tracker = DeepSort(
                max_age=5,
                n_init=2,
                nms_max_overlap=1.0,
                max_cosine_distance=0.3,
                nn_budget=None,
                override_track_class=None,
                embedder="mobilenet",
                half=True,
                bgr=True,
                embedder_gpu=True,
                embedder_model_name=None,
                embedder_wts=None,
                polygon=False,
                today=None,
            )
        self.get_logger().info(
            f"Camera {camera_id} YOLO{yolo_version} Tracker {tracker_enabled} Node has started"
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

    def formalize_detection(self, results, height, width):
        labels, cord = results
        detections = []
        n = len(labels)
        for i in range(n):
            row = cord[i]
            x1, y1, x2, y2 = (
                int(row[0] * width),
                int(row[1] * height),
                int(row[2] * width),
                int(row[3] * height),
            )
            score = float(row[4].item())
            name = self.detector.model.names[int(labels[i])]
            detections.append(([x1, y1, int(x2 - x1), int(y2 - y1)], score, name))
        return detections

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

            if self.tracker_enabled:
                # For tracker
                res = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
                detections = self.formalize_detection(
                    res, height=cv_image.shape[0], width=cv_image.shape[1]
                )
                tracks = self.tracker.update_tracks(detections, frame=cv_image)
                for track in tracks:
                    if not track.is_confirmed():
                        continue
                    track_id = track.track_id
                    ltrb = track.to_ltrb()
                    bbox = ltrb
                    cv2.putText(
                        cv_image,
                        "ID : " + str(track_id),
                        (int(bbox[0]) + 10, int(bbox[1] + 30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 0),
                        thickness=2,
                    )
                encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                    cv_image, "rgb8"
                )
            else:
                encoded_annotated_image = self._cv_bridge.cv2_to_imgmsg(
                    annotated_image, "rgb8"
                )

            self.publisher_annotated_image.publish(encoded_annotated_image)

            distance_calculator = DistanceCalculator()
            info_array = ClassCoordinatesArray()
            info_array.header.stamp = self.get_clock().now().to_msg()
            for result in new_results:
                # Informations part
                informations = ClassCoordinates()
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
    # Initialize ROS without passing args
    rclpy.init()

    # Create an argument parser for your script
    parser = argparse.ArgumentParser(description="ROS 2 YOLO Object Detection Node")

    # Add your custom argument
    parser.add_argument("--cam", type=str, default="1", help="Camera identifier")
    parser.add_argument("--yolo", type=str, default="v5", help="Yolo version")
    parser.add_argument(
        "--tracker", type=str, default="false", help="Enable tracker (false | true)"
    )

    # Parse the command line arguments
    custom_args = parser.parse_args()

    if custom_args.tracker == "true" or custom_args.tracker == "True":
        tracker = True
    else:
        tracker = False

    # Create and spin your node
    minimal_publisher = MinimalPublisher(custom_args.cam, custom_args.yolo, tracker)
    rclpy.spin(minimal_publisher)

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
