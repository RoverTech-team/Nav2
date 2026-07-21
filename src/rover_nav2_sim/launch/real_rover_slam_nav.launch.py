import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── numpy compat shim ──────────────────────────────────────────────
# Rimuove numpy 2.x da ~/.local che è incompatibile con rclpy su Humble.
# Conserva TUTTO ciò che viene da ros2_humble e dal workspace ros2_control.
_NUMPY_COMPAT_PATHS = [
    "/usr/lib/python3/dist-packages",
    "/opt/ros/humble/lib/python3/dist-packages",
    "/home/RoverTech/ros2_humble/local/lib/python3.10/dist-packages",
]

_CLEANED_PYTHONPATH = ":".join(
    p for p in sys.path
    if "site-packages" not in p                                      # rimuove site-packages generici
    or p in _NUMPY_COMPAT_PATHS                                      # salva path esplicitamente elencati
    or p.startswith("/opt/ros")                                      # salva ROS system
    or p.startswith("/usr")                                          # salva system Python
    or p.startswith("/home/RoverTech/ros2_humble")                   # salva TUTTO ros2_humble (rclpy, rpyutils, ecc.)
    or p.startswith("/home/RoverTech/ros2_control/install")          # salva workspace ros2_control
    or p.startswith("/home/RoverTech/nav2_sim_ws/install")           # salva workspace nav2_sim
)

_NUMPY_COMPAT_ENV = {}
if any(
    "site-packages" in p
    and p not in _NUMPY_COMPAT_PATHS
    and not p.startswith("/opt/ros")
    and not p.startswith("/usr")
    and not p.startswith("/home/RoverTech/ros2_humble")
    and not p.startswith("/home/RoverTech/ros2_control/install")
    and not p.startswith("/home/RoverTech/nav2_sim_ws/install")
    for p in sys.path
):
    _NUMPY_COMPAT_ENV["PYTHONPATH"] = _CLEANED_PYTHONPATH
# ── end shim ───────────────────────────────────────────────────────


def generate_launch_description():
    bringup_share = get_package_share_directory("my_robot_bringup")
    nav2_share    = get_package_share_directory("nav2_bringup")

    robot_launch      = os.path.join(bringup_share, "launch", "my_robot.launch.py")
    pointcloud_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan_nav2.yaml")
    slam_params       = os.path.join(bringup_share, "config", "slam_toolbox_nav2.yaml")
    nav2_params       = os.path.join(bringup_share, "config", "nav2_params_rover.yaml")
    default_map       = os.path.join(bringup_share, "maps", "rover_real_map.yaml")
    rviz_config       = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    mapping_mode = LaunchConfiguration("generate_new_map")

    return LaunchDescription([

        # ── Launch arguments ──────────────────────────────────────────────────
        DeclareLaunchArgument(
            "start_robot_bringup", default_value="true",
            description="Launch my_robot.launch.py (set false if already running)",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "generate_new_map", default_value="false",
            description=(
                "true  → SLAM costruisce la mappa live + Nav2 naviga in contemporanea "
                "        (SLAM pubblica /map e TF map->odom, no map_server, no amcl). "
                "false → Nav2 carica mappa da disco + AMCL per localizzazione."
            ),
        ),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/zed/zed_node/point_cloud/cloud_registered",
        ),
        DeclareLaunchArgument("scan_topic", default_value="/zed/scan"),

        # ── 1. Robot bringup ──────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "use_rviz":       "true",
                "camera_model":   "zed2i",
                "publish_tf":     "false",
                "publish_map_tf": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_robot_bringup")),
        ),

        # ── 2. Pointcloud → LaserScan ─────────────────────────────────────────
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[pointcloud_params],
            remappings=[
                ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                ("scan",     LaunchConfiguration("scan_topic")),
            ],
        ),

        # ── 3. SLAM Toolbox (sempre attivo) ───────────────────────────────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {
                    "use_sim_time": False,
                    "scan_topic":   LaunchConfiguration("scan_topic"),
                },
            ],
        ),

        # ── 4. Nav2 cmd-vel relay ─────────────────────────────────────────────
        Node(
            package="my_robot_bringup",
            executable="nav2_cmd_vel_relay.py",
            name="nav2_cmd_vel_relay",
            output="screen",
            env=_NUMPY_COMPAT_ENV,
            parameters=[{
                "input_topic":             "/cmd_vel",
                "output_topic":            "/diff_drive_controller/cmd_vel",
                "output_stamped":          True,
                "frame_id":                "base_footprint",
                "max_linear_x":            0.25,
                "max_angular_z":           0.50,
                "command_timeout":         0.5,
                "publish_zero_on_timeout": True,
            }],
        ),

        # ── 5. Map saver watchdog (solo mapping mode) ─────────────────────────
        Node(
            package="my_robot_bringup",
            executable="map_saver_watchdog.py",
            name="map_saver_watchdog",
            output="screen",
            env=_NUMPY_COMPAT_ENV,
            parameters=[{
                "map_yaml_path": LaunchConfiguration("map"),
                "poll_interval": 1.0,
                "timeout":       120.0,
                "save_delay":    2.0,
            }],
            condition=IfCondition(mapping_mode),
        ),

        # =====================================================================
        # Nav2 — MODO A: generate_new_map=true
        #
        # SLAM pubblica /map e TF map->odom.
        # Saltiamo map_server e amcl — Nav2 naviga sulla mappa live.
        # =====================================================================

        TimerAction(period=4.0, actions=[Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=6.0, actions=[Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=8.0, actions=[Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=10.0, actions=[Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=12.0, actions=[Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=14.0, actions=[Node(
            package="nav2_waypoint_follower", executable="waypoint_follower",
            name="waypoint_follower", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        TimerAction(period=16.0, actions=[Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=IfCondition(mapping_mode),
        )]),

        # Lifecycle manager — MODO A (senza map_server e amcl)
        TimerAction(period=18.0, actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart":    True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                    "smoother_server",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }],
            condition=IfCondition(mapping_mode),
        )]),

        # =====================================================================
        # Nav2 — MODO B: generate_new_map=false
        #
        # Mappa caricata da disco + AMCL. Stack Nav2 completo.
        # =====================================================================

        TimerAction(period=0.1, actions=[Node(
            package="nav2_map_server", executable="map_server",
            name="map_server", output="screen",
            parameters=[nav2_params, {
                "use_sim_time":  False,
                "yaml_filename": LaunchConfiguration("map"),
            }],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=2.0, actions=[Node(
            package="nav2_amcl", executable="amcl",
            name="amcl", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=4.0, actions=[Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=6.0, actions=[Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=8.0, actions=[Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=10.0, actions=[Node(
            package="nav2_behaviors", executable="behavior_server",
            name="behavior_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=12.0, actions=[Node(
            package="nav2_smoother", executable="smoother_server",
            name="smoother_server", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=14.0, actions=[Node(
            package="nav2_waypoint_follower", executable="waypoint_follower",
            name="waypoint_follower", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        TimerAction(period=16.0, actions=[Node(
            package="nav2_velocity_smoother", executable="velocity_smoother",
            name="velocity_smoother", output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
            condition=UnlessCondition(mapping_mode),
        )]),

        # Lifecycle manager — MODO B (stack completo)
        TimerAction(period=18.0, actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart":    True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "behavior_server",
                    "smoother_server",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }],
            condition=UnlessCondition(mapping_mode),
        )]),

        # ── 7. RViz2 ──────────────────────────────────────────────────────────
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
