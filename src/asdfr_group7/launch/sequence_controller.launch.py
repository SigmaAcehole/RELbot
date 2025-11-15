
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node, RosTimer

def generate_launch_description():
    # -------- Launched actions --------
    
    dynamic: DeclareLaunchArgument = DeclareLaunchArgument(
        name="dynamic",
        default_value="true",
        description="Whether to use light position command mode. If set to false, uses default setpoints in sequence controller.",
    )


    brightness_threshold_arg: DeclareLaunchArgument = DeclareLaunchArgument(
        name="brightness_threshold_arg",
        default_value="128",
        description="Threshold to brightness argument. By default, it is 128.",
    )

    depth: DeclareLaunchArgument = DeclareLaunchArgument(
        name="depth",
        default_value="1",
        description="Queue depth"
    )

    show_camera: DeclareLaunchArgument = DeclareLaunchArgument(
        name="show_camera",
        default_value="false",
        description="show camera"
    )

    debug: DeclareLaunchArgument = DeclareLaunchArgument(
        name="debug",
        default_value="false",
        description="show camera log"
    )

    reliability: DeclareLaunchArgument = DeclareLaunchArgument(
        name="reliability",
        default_value="reliable",
        choices=["reliable", "best_effort"],
        description="Reliabilty of the camera"
    )

    history: DeclareLaunchArgument = DeclareLaunchArgument(
        name="history",
        default_value="keep_last",
        choices=["keep_last", "keep_all"],
        description="Message keep history. Default keep all"
    )

    return LaunchDescription([
        show_camera,
        brightness_threshold_arg, dynamic,
        depth, reliability, history, debug,
        Node(
            package='image_tools_sdfr',
            executable='cam2image',
            name='cam2image',
                parameters=[
                    # Set the command mode based on the launch argument
                    {"depth": LaunchConfiguration(depth.name)},
                    {"reliability": LaunchConfiguration(reliability.name)},
                    {"history": LaunchConfiguration(history.name)},
                    {"show_camera": LaunchConfiguration(show_camera.name)},
                    {"debug": LaunchConfiguration(debug.name)}
                ],
        ),
        Node(
            package='asdfr_group7',
            executable='lightposition',
            name='lightposition',
                parameters=[
                    # Set the command mode based on the launch argument
                    {"brightness_threshold_arg": LaunchConfiguration(brightness_threshold_arg.name)},
                    {"depth": LaunchConfiguration(depth.name)},
                    {"reliability": LaunchConfiguration(reliability.name)},
                    {"history": LaunchConfiguration(history.name)},
                ],
        ),
        RosTimer(period=2.0, actions=[Node(
            package='asdfr_group7',
            executable='sequence_controller',
            name='sequence_controller',
                parameters=[
                    {"depth": LaunchConfiguration(depth.name)},
                    {"reliability": LaunchConfiguration(reliability.name)},
                    {"history": LaunchConfiguration(history.name)},
                    {"dynamic": LaunchConfiguration(dynamic.name)}
                ],
        )])
    ])
