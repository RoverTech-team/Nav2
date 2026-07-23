import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aruco_detection"),
        "config",
        "zed_params.yaml",
    )

    rviz_config = os.path.join(
        get_package_share_directory("aruco_detection"),
        "config",
        "zed_detection.rviz",
    )

    zed_override = os.path.join(
        get_package_share_directory("aruco_detection"),
        "config",
        "zed_override.yaml",
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "ros_params_override_path": zed_override,
        }.items(),
    )

    return LaunchDescription([
        zed_launch,
        Node(
            package="aruco_detection",
            executable="detection_node",
            name="aruco_detection_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
        ),
    ])
