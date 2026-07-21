import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_nav2_sim")
    nav2_share = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    rviz_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")
    default_map = os.path.join(pkg_share, "maps", "rover_test_map.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Absolute path to the map YAML generated during the mapping step.",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "sim.launch.py"))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": "true",
                    "params_file": params_file,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
