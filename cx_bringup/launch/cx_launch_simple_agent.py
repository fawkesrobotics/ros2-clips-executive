# Licensed under GPLv2. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_with_context(context, *args, **kwargs):
    bringup_dir = get_package_share_directory("cx_bringup")

    namespace = LaunchConfiguration("namespace")
    manager_config = LaunchConfiguration("manager_config")
    manager_config_file = os.path.join(bringup_dir, "params", manager_config.perform(context))
    log_level = LaunchConfiguration("log_level")
    cx_node = Node(
        package="cx_bringup",
        executable="cx_node",
        output="screen",
        emulate_tty=True,
        namespace=namespace,
        parameters=[
            manager_config_file,
        ],
        arguments=["--ros-args", "--log-level", log_level],
        # arguments=[('--ros-args --log-level debug')]
    )
    return [cx_node]


def generate_launch_description():

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_manager_config = DeclareLaunchArgument(
        "manager_config",
        default_value="clips_env_manager.yaml",
        description="Name of the CLIPS environment manager configuration",
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_manager_config)
    ld.add_action(OpaqueFunction(function=launch_with_context))

    return ld
