"""Starts the whole system, then automatically sends a goal to /draw_square."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory('turtle_motion'), 'launch', 'system.launch.py')

    send_square = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/draw_square',
             'turtle_interfaces/action/DrawSquare',
             ['{side_length: ', LaunchConfiguration('side_length'), '}'],
             '--feedback'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='3.0',
                              description='Square side length (0 < side <= 5.3)'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(system_launch)),
        # give turtlesim and the servers a few seconds to come up
        TimerAction(period=4.0, actions=[send_square]),
    ])
