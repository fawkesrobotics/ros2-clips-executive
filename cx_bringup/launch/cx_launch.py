import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('cx_bringup')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_ = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Default namespace')

    declare_params_file_ = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'cx_params.yaml'),
        description='Path to the ROS2 parameters file')

    cx_clips_node_ = Node(
        package='cx_clips',
        executable='clips_node',
        name='clips_manager',
        output='screen',
        namespace=namespace,
        parameters=[params_file],
    )

    cx_features_node_ = Node(
        package='cx_features',
        executable='features_node',
        name='clips_features_manager',
        output='screen',
        namespace=namespace,
        parameters=[params_file],
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(declare_namespace_)
    ld.add_action(declare_params_file_)

    ld.add_action(cx_clips_node_)
    ld.add_action(cx_features_node_)

    return ld
