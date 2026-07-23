import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("aruco_detection"),
        "config",
        "zed_params.yaml",
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed_wrapper"),
                "launch",
                "zed_camera.launch.py",
            )
        ),
        launch_arguments={"camera_model": "zed2i"}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("test_duration", default_value="120.0"),
        DeclareLaunchArgument("output_dir", default_value="~/aruco_test_results"),
        DeclareLaunchArgument("marker_azimuths", default_value='{"0": 90.0, "1": 180.0, "42": 270.0}'),
        DeclareLaunchArgument("yaw_threshold", default_value="10.0"),

        zed_launch,
        Node(
            package="aruco_detection",
            executable="detection_node",
            name="aruco_detection_node",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="aruco_detection",
            executable="yaw_test_node",
            name="aruco_yaw_test_node",
            output="screen",
            parameters=[{
                "marker_azimuths": ParameterValue(
                    LaunchConfiguration("marker_azimuths"), value_type=str
                ),
                "test_duration": ParameterValue(
                    LaunchConfiguration("test_duration"), value_type=float
                ),
                "output_dir": ParameterValue(
                    LaunchConfiguration("output_dir"), value_type=str
                ),
                "yaw_threshold": ParameterValue(
                    LaunchConfiguration("yaw_threshold"), value_type=float
                ),
            }],
        ),
    ])
