from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the serial bridge node."""
    return LaunchDescription([
        Node(
            package='comunication_serial',
            executable='comunication_serial_executable',
            output='screen')
    ])
