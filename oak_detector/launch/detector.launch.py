from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ir_projector_brightness', default_value='0.0', description='from 0.0 to 1200.0 mA'),
        DeclareLaunchArgument('nn_blob_path', default_value=''),
        Node(
            package='oak_detector',
            namespace='oak_detector',
            executable='oak_detector_node',
            name='oak_detector_node',
            parameters=[
                {'ir_projector_brightness': LaunchConfiguration('ir_projector_brightness')},
                {'nn_blob_path': LaunchConfiguration('nn_blob_path')}
            ]
        ),
    ])
