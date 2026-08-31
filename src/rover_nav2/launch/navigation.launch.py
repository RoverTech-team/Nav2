import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _numpy_compat_env():
    """Keep user-site numpy 2.x out of rclpy processes on ROS 2 Humble."""
    allowed_exact = {
        "/usr/lib/python3/dist-packages",
        "/opt/ros/humble/lib/python3/dist-packages",
        "/home/RoverTech/ros2_humble/local/lib/python3.10/dist-packages",
    }
    allowed_prefixes = (
        "/opt/ros",
        "/usr",
        "/home/RoverTech/ros2_humble",
        "/home/RoverTech/nav2_ws/install",
        "/home/RoverTech/nav2_ws_new/install",
        "/home/RoverTech/nav2_ws_new",
    )

    cleaned = [
        path
        for path in sys.path
        if "site-packages" not in path
        or path in allowed_exact
        or path.startswith(allowed_prefixes)
    ]

    if cleaned == sys.path:
        return {}
    return {"PYTHONPATH": ":".join(cleaned)}


_NUMPY_COMPAT_ENV = _numpy_compat_env()


def _timed_node(package, executable, name, params_file, condition, delay, remappings=None):
    return TimerAction(
        period=delay,
        actions=[
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=[params_file, {"use_sim_time": False}],
                remappings=remappings or [],
                condition=condition,
            )
        ],
    )


def _navigation_nodes(params_file, condition):
    return [
        _timed_node(
            "nav2_controller",
            "controller_server",
            "controller_server",
            params_file,
            condition,
            4.0,
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        _timed_node("nav2_planner", "planner_server", "planner_server", params_file, condition, 6.0),
        _timed_node("nav2_bt_navigator", "bt_navigator", "bt_navigator", params_file, condition, 8.0),
        _timed_node("nav2_behaviors", "behavior_server", "behavior_server", params_file, condition, 10.0),
        _timed_node("nav2_smoother", "smoother_server", "smoother_server", params_file, condition, 12.0),
        _timed_node("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower", params_file, condition, 14.0),
        _timed_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            "velocity_smoother",
            params_file,
            condition,
            16.0,
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
    ]


def _lifecycle_manager(node_names, condition, delay=18.0):
    return TimerAction(
        period=delay,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": node_names,
                }],
                condition=condition,
            )
        ],
    )


def generate_launch_description():
    bringup_share = get_package_share_directory("my_robot_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")

    robot_launch = os.path.join(bringup_share, "launch", "my_robot.launch.py")
    pointcloud_params = os.path.join(bringup_share, "config", "pointcloud_to_laserscan_nav2.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox_nav2.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params_rover.yaml")
    ekf_params = os.path.join(bringup_share, "config", "ekf_params.yaml")
    aruco_share = get_package_share_directory("aruco_detection")
    zed_params = os.path.join(aruco_share, "config", "zed_params.yaml")
    attitude_params = os.path.join(aruco_share, "config", "attitude_params.yaml")
    rviz_config = os.path.join(nav2_share, "rviz", "nav2_default_view.rviz")

    # FIX: handle both ws locations; prefer file that actually exists
    candidate_dirs = [
        os.path.join(os.path.expanduser("~"), "nav2_ws_new", "maps"),
        os.path.join(os.path.expanduser("~"), "nav2_ws", "maps"),
        os.path.join(bringup_share, "maps"),
    ]
    default_map = None
    for d in candidate_dirs:
        candidate = os.path.join(d, "rover_real_map.yaml")
        if os.path.isfile(candidate):
            default_map = candidate
            map_dir = d
            break
    if default_map is None:
        map_dir = next((d for d in candidate_dirs if os.path.isdir(d) and os.listdir(d)), candidate_dirs[0])
        os.makedirs(map_dir, exist_ok=True)
        default_map = os.path.join(map_dir, "rover_real_map.yaml")
    if not os.path.isfile(default_map):
        print(f"[navigation.launch.py] WARNING: default map not found at {default_map}")

    mapping_mode = LaunchConfiguration("generate_new_map")
    mapping_condition = IfCondition(mapping_mode)
    localization_condition = UnlessCondition(mapping_mode)
    auto_save_condition = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("generate_new_map"), "' == 'true' and '",
            LaunchConfiguration("auto_save_map"), "' == 'true'"
        ])
    )

    mapping_nodes = [
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "smoother_server",
        "waypoint_follower",
        "velocity_smoother",
    ]
    localization_nodes = ["map_server", "amcl", *mapping_nodes]

    # ── Phase 2b: ArUco detection → attitude (Kalman-filtered yaw) → EKF fusion ──
    aruco_nodes = [
        Node(
            package="aruco_detection",
            executable="detection_node",
            name="aruco_detection_node",
            output="screen",
            parameters=[zed_params],
            additional_env=_NUMPY_COMPAT_ENV,
        ),
        Node(
            package="aruco_detection",
            executable="attitude_node",
            name="aruco_attitude_node",
            output="screen",
            parameters=[attitude_params],
            additional_env=_NUMPY_COMPAT_ENV,
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_robot_bringup",
                default_value="true",
                description="Launch my_robot.launch.py (set false if already running)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "generate_new_map",
                default_value="true",
                description=(
                    "true runs SLAM and Nav2 on the live map. "
                    "false loads the map argument and localizes with AMCL."
                ),
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("auto_save_map", default_value="true"),
            DeclareLaunchArgument("map_save_initial_delay", default_value="30.0"),
            DeclareLaunchArgument("map_save_period", default_value="30.0"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/zed/zed_node/point_cloud/cloud_registered",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/zed/scan"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={"use_rviz": "false"}.items(),
                condition=IfCondition(LaunchConfiguration("start_robot_bringup")),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[pointcloud_params],
                remappings=[
                    ("cloud_in", LaunchConfiguration("pointcloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
            *aruco_nodes,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=mapping_condition,
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params,
                    {
                        "use_sim_time": False,
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "use_lifecycle_manager": False,
                    },
                ],
                condition=mapping_condition,
            ),
            Node(
                package="my_robot_bringup",
                executable="nav2_cmd_vel_relay.py",
                name="nav2_cmd_vel_relay",
                output="screen",
                additional_env=_NUMPY_COMPAT_ENV,
                parameters=[{
                    "input_topic": "/cmd_vel",
                    "output_topic": "/diff_drive_controller/cmd_vel",
                    "output_stamped": True,
                    "frame_id": "base_footprint",
                    "max_linear_x": 0.25,
                    "max_angular_z": 0.50,
                    "command_timeout": 0.5,
                    "publish_zero_on_timeout": True,
                }],
            ),
            Node(
                package="my_robot_bringup",
                executable="map_saver_watchdog.py",
                name="map_saver_watchdog",
                output="screen",
                additional_env=_NUMPY_COMPAT_ENV,
                parameters=[{
                    "map_yaml_path": LaunchConfiguration("map"),
                    "initial_delay": LaunchConfiguration("map_save_initial_delay"),
                    "save_period": LaunchConfiguration("map_save_period"),
                }],
                condition=auto_save_condition,
            ),
            *_navigation_nodes(nav2_params, mapping_condition),
            _lifecycle_manager(mapping_nodes, mapping_condition, 35.0),
            TimerAction(
                period=0.1,
                actions=[
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            nav2_params,
                            {
                                "use_sim_time": False,
                                "yaml_filename": LaunchConfiguration("map"),
                            },
                        ],
                        condition=localization_condition,
                    )
                ],
            ),
            _timed_node("nav2_amcl", "amcl", "amcl", nav2_params, localization_condition, 2.0),
            *_navigation_nodes(nav2_params, localization_condition),
            _lifecycle_manager(localization_nodes, localization_condition),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": False}],
                additional_env={"DISPLAY": os.environ.get("DISPLAY", ":1")},
            ),
        ]
    )
