from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("rm_ros2_manual"),
        'cfg',
        'manual.yaml'
    )
    return LaunchDescription([
        Node(
            package='rm_ros2_manual',
            executable='rm_ros2_manual',
            name='rm_ros2_manual',
            parameters=[config]
        )
    ])
