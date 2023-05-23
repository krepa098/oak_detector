from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oak_detector_vis',
            namespace='oak_detector_vis',
            executable='oak_detector_vis_node',
            name='oak_detector_vis_node'
        ),
    ])
