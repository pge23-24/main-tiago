import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from src.communication.sender import Sender

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'Cam1/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self._cv_bridge = CvBridge()

        self.sender = Sender()

    def listener_callback(self, image):
        self.get_logger().info("image recieved")
        self.cv_image = cv2.cvtColor(self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough"), cv2.COLOR_BGR2RGB)

        self.sender.send_data(self.cv_image)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




