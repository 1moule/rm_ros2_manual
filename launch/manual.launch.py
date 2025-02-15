from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    manual_config = os.path.join(
        get_package_share_directory("rm_ros2_manual"),
        'cfg',
        'manual.yaml'
    )
    controller_config = os.path.join(
        get_package_share_directory("rm_description"),
        'cfg',
        'test.yaml'
    )
    return LaunchDescription([
        Node(
            package='rm_ros2_manual',
            executable='rm_ros2_manual',
            name='rm_ros2_manual',
            parameters=[manual_config, controller_config],
            output='screen',
            prefix=['gdb -ex run --args'],
        )
    ])
