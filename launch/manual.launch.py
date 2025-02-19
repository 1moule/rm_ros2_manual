import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # param
    declared_arguments = [
        DeclareLaunchArgument(
            name='odom_tf',
            default_value='true',
            description='Whether to publish the odom tf'
        )]

    # config
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

    # description
    description_path = os.path.join(
        get_package_share_directory('rm_description'))

    xacro_file = os.path.join(description_path,
                              'urdf',
                              os.getenv("ROBOT_TYPE"),
                              os.getenv("ROBOT_TYPE") + '.urdf.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # nodes
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[controller_config],
        output="screen"
    )
    manual = Node(
        package='rm_ros2_manual',
        executable='rm_ros2_manual',
        name='rm_ros2_manual',
        parameters=[manual_config],
        output='screen'
    )
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='/controller_manager',
        parameters=[params],
        output='screen'
    )
    static_transform_publisher = Node(
        condition=IfCondition(LaunchConfiguration('odom_tf')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom', '--ros-args', '--param',
                   'publish_period_sec:=0.01']
    )

    return LaunchDescription(declared_arguments + [
        controller_manager,
        manual,
        node_robot_state_publisher,
        static_transform_publisher,
    ])
