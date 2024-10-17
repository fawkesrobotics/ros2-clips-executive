# Licensed under GPLv2. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("cx_bringup")

    namespace = LaunchConfiguration("namespace")
    clips_env_manager_file = LaunchConfiguration("clips_env_manager_file")
    log_level = LaunchConfiguration("log_level")

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_clips_env_manager_file = DeclareLaunchArgument(
        "clips_env_manager_file",
        default_value=os.path.join(bringup_dir, "params", "clips_env_manager.yaml"),
        description="Path to the ROS2 env manager file",
    )

    cx_node = Node(
        package="cx_bringup",
        executable="cx_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[
            clips_env_manager_file,
        ],
        arguments=["--ros-args", "--log-level", log_level],
        # arguments=[('--ros-args --log-level debug')]
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_clips_env_manager_file)
    ld.add_action(cx_node)
    return ld
