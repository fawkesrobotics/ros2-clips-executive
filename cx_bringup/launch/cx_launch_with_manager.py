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

    lc_nodes = ["clips_manager",
                "clips_features_manager"]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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

    cx_lifecycle_manager = Node(
        package='cx_lifecycle_nodes_manager',
        executable='lifecycle_manager_node',
        name='cx_lifecycle_manager',
        output='screen',
        namespace=namespace,
        parameters=[{"node_names_to_manage": lc_nodes}],
    )

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_params_file_)

    ld.add_action(cx_clips_node_)
    ld.add_action(cx_features_node_)
    ld.add_action(cx_lifecycle_manager)

    return ld
