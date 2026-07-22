import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aruco_detection_dev"),
        "config",
        "webcam_params.yaml",
    )

    return LaunchDescription([
        Node(
            package="aruco_detection_dev",
            executable="detection_node",
            name="aruco_detection_node",
            output="screen",
            parameters=[config],
        ),
    ])
