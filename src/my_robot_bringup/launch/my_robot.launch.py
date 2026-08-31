from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path, get_package_share_directory
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    controllers_config_path = os.path.join(get_package_share_directory("my_robot_bringup"), "config", "my_robot_controller.yaml")

    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'),
                                    'rviz', 'urdf_config.rviz')

    aruco_share = get_package_share_directory("aruco_detection")
    zed_override = os.path.join(aruco_share, "config", "zed_override.yaml")

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    ros2_control_node = TimerAction(
        period=3.0,   # aspetta RSP
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                output='screen',
                parameters=[controllers_config_path],
                remappings=[('/controller_manager/robot_description', '/robot_description')]
            )]
    )

    joint_state_broadcaster_spawner = TimerAction(
	period=5.0,
	actions=[Node(
        	package='controller_manager',
        	executable='spawner',
        	arguments=['joint_state_broadcaster'])]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    diff_drive_controller_spawner = TimerAction(
    period=6.0,
    actions=[Node(
        	package='controller_manager',
        	executable='spawner',
        	arguments=['diff_drive_controller'])]
    )

    # Auto brake release: pulse reset_fault 1s on every launch, watchdog every 5s until OP_ENABLED
    brake_release_controller_spawner = TimerAction(
        period=6.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['brake_release_controller'],
        )]
    )

    brake_release_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='my_robot_bringup',
            executable='brake_release_node.py',
            name='brake_release_node',
            output='screen',
            parameters=[{
                'pulse_duration': 1.0,
                'repeat_interval': 5.0,
                'auto_repeat': True,
                'activate_controller': False,
            }],
        )]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Get the ZED wrapper launch file path
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'publish_urdf': 'false',
            'ros_params_override_path': zed_override,
        }.items()
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_state_publisher_gui_node,
        diff_drive_controller_spawner,
        brake_release_controller_spawner,
        brake_release_node,
        rviz2_node,
        zed_launch
    ])
