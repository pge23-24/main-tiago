import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
from rclpy.utilities import remove_ros_args
import sys


class ImageDisplayNode(Node):
    def __init__(self, topic_name):
        self.topic_name = topic_name
        super().__init__("image_display_node")
        self.subscription = self.create_subscription(
            Image, self.topic_name, self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()
        self.record_image_path = "/media/pge-2023/DDE DORIAN/PGE/video_1/"
        self.i = 0

    def listener_callback(self, image):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        cv_rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow(f"Recorded Image {self.topic_name}", cv_rgb_image)
        # Record vers le fichier souhaité
        cv2.imwrite(
            self.record_image_path + "image_" + str(self.i) + ".png", cv_rgb_image
        )
        # cv2.imshow(f"Recorded Image {self.topic_name}", cv_rgb_image)
        self.i += 1
        # cv2.waitKey(1)


def main(args=None):
    # Initialize ROS without passing args
    rclpy.init()

    # Create an argument parser for your script
    parser = argparse.ArgumentParser(description="Image Display Node")

    # Add your custom argument
    parser.add_argument(
        "--topic", type=str, default="1", required=True, help="Topic to subscribe to"
    )

    # Parse the command line arguments
    custom_args = parser.parse_args()

    # Create and spin your node
    image_display_node = ImageDisplayNode(topic_name=custom_args.topic)
    rclpy.spin(image_display_node)

    # Shutdown and cleanup
    image_display_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
