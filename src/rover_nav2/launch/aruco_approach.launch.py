import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_map():
    """Same map auto-discovery as navigation.launch.py, so the `map` argument
    can always be passed through explicitly."""
    bringup_share = get_package_share_directory("my_robot_bringup")
    candidate_dirs = [
        os.path.join(os.path.expanduser("~"), "nav2_ws_new", "maps"),
        os.path.join(os.path.expanduser("~"), "nav2_ws", "maps"),
        os.path.join(bringup_share, "maps"),
    ]
    for d in candidate_dirs:
        candidate = os.path.join(d, "rover_real_map.yaml")
        if os.path.isfile(candidate):
            return candidate
    # Fallback: first dir that exists and is non-empty, or first candidate
    map_dir = next((d for d in candidate_dirs if os.path.isdir(d) and os.listdir(d)), candidate_dirs[0])
    return os.path.join(map_dir, "rover_real_map.yaml")


def generate_launch_description():
    bringup_share = get_package_share_directory("my_robot_bringup")

    params_file = os.path.join(bringup_share, "config", "marker_navigator.yaml")
    default_map = _default_map()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_robot_bringup",
                default_value="true",
                description="Launch my_robot.launch.py (set false if already running)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "generate_new_map",
                default_value="false",
                description="true runs SLAM; marker approach defaults to AMCL localization",
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("marker_id", default_value="42"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("rover_nav2"),
                        "launch",
                        "navigation.launch.py",
                    )
                ),
                launch_arguments={
                    "start_robot_bringup": LaunchConfiguration("start_robot_bringup"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "generate_new_map": LaunchConfiguration("generate_new_map"),
                    "map": LaunchConfiguration("map"),
                }.items(),
            ),
            # navigation.launch.py stages its Nav2 servers with delays up to
            # ~18 s; wait for the behavior server before the first explore
            # action (the node also waits for the action server itself).
            TimerAction(
                period=20.0,
                actions=[
                    Node(
                        package="my_robot_bringup",
                        executable="marker_navigator_node.py",
                        name="marker_navigator_node",
                        output="screen",
                        parameters=[
                            params_file,
                            {"target_marker_id": LaunchConfiguration("marker_id")},
                        ],
                    )
                ],
            ),
        ]
    )
