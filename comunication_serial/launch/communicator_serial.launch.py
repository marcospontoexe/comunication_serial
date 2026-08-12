from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comunication_serial',
            executable='comunication_serial_executable',
            output='screen')
    ])