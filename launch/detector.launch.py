from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oak_detector',
            namespace='oak_detector',
            executable='oak_detector_node',
            name='oak_detector_node'
        ),
    ])
