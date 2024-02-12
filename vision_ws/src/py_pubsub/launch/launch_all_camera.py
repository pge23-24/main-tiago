import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    for cam_id in range(1, 5):
        node = Node(
            package='py_pubsub',
            executable='camera_yolo',
            name=f'camera_yolo_{cam_id}',
            parameters=[
                {"cam_id": cam_id}
            ]
        )
        ld.add_action(node)

    return ld

if __name__ == '__main__':
    generate_launch_description()