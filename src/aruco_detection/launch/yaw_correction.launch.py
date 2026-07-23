import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    attitude_config = os.path.join(
        get_package_share_directory("aruco_detection"),
        "config",
        "attitude_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="aruco_detection",
            executable="attitude_node",
            name="aruco_attitude_node",
            output="screen",
            parameters=[attitude_config],
        ),
    ])
