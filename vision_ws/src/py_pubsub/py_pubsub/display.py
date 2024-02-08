import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__("image_display_node")
        self.declare_parameter('image_topic_id', 1)

        self.topic_id = self.get_parameter('image_topic_id').get_parameter_value().integer_value
        
        self.topic_name= "annotated_images_" + str(self.topic_id)

        self.subscription = self.create_subscription(
            Image, self.topic_name, self.listener_callback, 10
        )
        self._cv_bridge = CvBridge()

        self.get_logger().info(f"Displaying images from topic {self.topic_name}")

    def listener_callback(self, image):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
        cv2.imshow(f"Annotated Image {self.topic_name}", cv_image)
        cv2.waitKey(1)


def main(args=None):
    # Initialize ROS without passing args
    rclpy.init(args=args)

    # Create and spin your node
    image_display_node = ImageDisplayNode()
    rclpy.spin(image_display_node)

    # Shutdown and cleanup
    image_display_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
