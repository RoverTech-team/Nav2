import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn_rover(context, *args, **kwargs):
    import xacro

    pkg_share = get_package_share_directory("rover_nav2_sim")
    model_xacro = os.path.join(pkg_share, "models", "rover_sim.sdf.xacro")
    world_name = LaunchConfiguration("world_name").perform(context)

    rendered = xacro.process_file(model_xacro).toxml()
    render_dir = os.path.join(tempfile.gettempdir(), "rover_nav2_sim")
    os.makedirs(render_dir, exist_ok=True)
    rendered_sdf = os.path.join(render_dir, "rover_sim.sdf")
    with open(rendered_sdf, "w", encoding="utf-8") as sdf_file:
        sdf_file.write(rendered)

    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-world",
                world_name,
                "-file",
                rendered_sdf,
                "-name",
                "rover",
                "-x",
                "0.0",
                "-y",
                "0.0",
                "-z",
                "0.02",
            ],
        )
    ]


def static_tf(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x",
            str(xyz[0]),
            "--y",
            str(xyz[1]),
            "--z",
            str(xyz[2]),
            "--roll",
            str(rpy[0]),
            "--pitch",
            str(rpy[1]),
            "--yaw",
            str(rpy[2]),
            "--frame-id",
            parent,
            "--child-frame-id",
            child,
        ],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_nav2_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    default_world = os.path.join(pkg_share, "worlds", "rover_test_world.sdf")
    bridge_config = os.path.join(pkg_share, "config", "bridge_config.yaml")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("world_name", default_value="rover_test_world"),
            gz_sim,
            bridge,
            static_tf("base_to_body_tf", "base_footprint", "base_link", (0.0, 0.0, 0.10)),
            static_tf("base_to_lidar_tf", "base_link", "lidar_link", (0.2025, 0.0, 0.34)),
            static_tf("base_to_zed_tf", "base_link", "zed_camera_link", (0.2025, 0.0, 0.54)),
            TimerAction(period=2.0, actions=[OpaqueFunction(function=spawn_rover)]),
        ]
    )
