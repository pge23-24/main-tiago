import rclpy
from rclpy.node import Node
from py_pubsub_msgs.msg import ClassCoordinates

from std_msgs.msg import String
import argparse



class MinimalPublisher(Node):
    def __init__(self, camera_id):
        super().__init__("minimal_publisher")
        self.camera_id = camera_id

        self.topic_name_information = f"information_{camera_id}"
        self.publisher_information = self.create_publisher(
            ClassCoordinates, self.topic_name_information, 10
        )

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
    
    while(rclpy.ok()):
        print("Publishing")
        msg = ClassCoordinates()
        msg.header.stamp = minimal_publisher.get_clock().now().to_msg()
        
        minimal_publisher.publisher_information.publish(msg)
        rclpy.sleep(1)

    # Shutdown and cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
