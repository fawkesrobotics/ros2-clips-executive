import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('cx_bringup')
    cx_dir = get_package_share_directory('cx_clips_executive')

    namespace = LaunchConfiguration('namespace')
    cx_params_file = LaunchConfiguration('cx_params_file')
    log_level = LaunchConfiguration('log_level')
    model_file = LaunchConfiguration('model_file')

    clips_executive_params_file = LaunchConfiguration(
        'clips_executive_params_file')

    lc_nodes = ["clips_features_manager", "clips_executive"]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    declare_model_file_cmd = DeclareLaunchArgument(
        'model_file',
        default_value=os.path.join(cx_dir + "/clips/test-scenario-nav2/domain.pddl"),
        description='PDDL Model file')

    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value='info',
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Default namespace')

    declare_cx_params_file = DeclareLaunchArgument(
        'cx_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'clips_features_manager.yaml'),
        description='Path to the ROS2 cx_params.yaml file')

    declare_clips_executive_params_file = DeclareLaunchArgument(
        'clips_executive_params_file',
        default_value=os.path.join(
            bringup_dir, 'params', 'simple_agent.yaml'),
        description='Path to Clips Executive params file')

    cx_node = Node(
        package='cx_bringup',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        namespace=namespace,
        parameters=[
            cx_params_file,
            clips_executive_params_file,
            {"agent_dir":bringup_dir},
            {"clips_executive_config": clips_executive_params_file},
            {"clips_features_manager_config": cx_params_file},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    cx_lifecycle_manager = Node(
        package='cx_lifecycle_nodes_manager',
        executable='lifecycle_manager_node',
        name='cx_lifecycle_manager',
        output='screen',
        emulate_tty=True,
        namespace=namespace,
        parameters=[{"node_names_to_manage": lc_nodes}]
    )

    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_cx_params_file)
    ld.add_action(declare_clips_executive_params_file)
    ld.add_action(declare_model_file_cmd)

    ld.add_action(cx_node)
    ld.add_action(cx_lifecycle_manager)

    return ld
