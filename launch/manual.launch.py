from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_ros2_manual',
            executable='rm_ros2_manual',
            name='rm_ros2_manual'
        )
    ])
