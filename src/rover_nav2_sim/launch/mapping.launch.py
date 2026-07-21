import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_nav2_sim")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "sim.launch.py"))
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "slam_toolbox.yaml"),
                    {"use_sim_time": True, "scan_topic": "/scan"},
                ],
            ),
        ]
    )
