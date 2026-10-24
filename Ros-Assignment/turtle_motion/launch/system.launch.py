"""Starts turtlesim and every node of the lab, with speed limits from a YAML file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('turtle_motion'), 'config', 'speed_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with max_linear_speed / max_angular_speed'),

        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),

        Node(package='turtle_motion', executable='go_to_pose_server',
             name='go_to_pose_server', output='screen',
             parameters=[params_file]),

        Node(package='turtle_motion', executable='reset_service',
             name='reset_service', output='screen'),

        Node(package='turtle_motion', executable='square_action_server',
             name='square_action_server', output='screen'),
    ])
