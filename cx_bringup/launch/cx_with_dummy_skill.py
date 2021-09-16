import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('cx_bringup')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    dummy_move_node = Node(
        package='cx_example_skill_nodes',
        executable='skills_launch_node',
        name='dummy_move_skill',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    cx_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir,
            'launch',
            'cx_launch.py')),
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(dummy_move_node)
    ld.add_action(cx_bringup)

    return ld
