import rclpy
from rclpy.timer import Rate
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("simu_pub")
        self.publisher = self.create_publisher(Image, "Cam1/image_raw", 10)
        self._cv_bridge = CvBridge()
        self.cv_image = cv2.imread("src/py_pubsub/resource/cario_image.png")
        self.timer_rate = self.create_rate(20)

    def publish(self):
        while rclpy.ok():

            image_message = self._cv_bridge.cv2_to_imgmsg(
                self.cv_image, encoding="rgb8"
            )
            self.get_logger().info("Publishing")
            self.publisher.publish(image_message)
            rclpy.spin_once(self)
            self.timer_rate.sleep()


def main(args=None):
    # Initialize ROS without passing args
    rclpy.init()

    # Create and spin your node
    minimal_publisher = MinimalPublisher()

    try:
        minimal_publisher.publish()
    except KeyboardInterrupt:
        pass

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
