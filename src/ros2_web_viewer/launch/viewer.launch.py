"""Launch file for ros2_web_viewer.

Parameter configuration:
  - All parameters are loaded from a YAML configuration file (config/params.yaml by default).
  - Optionally, override the config file using the 'params_file' launch argument:
      ros2 launch ros2_web_viewer viewer.launch.py params_file:=path/to/custom_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_web_viewer')
    default_params_file = PathJoinSubstitution([pkg_dir, 'config', 'params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=[default_params_file],
            description='Path to ROS2 parameters YAML file (config/params.yaml by default)'),

        Node(
            package='ros2_web_viewer',
            executable='ros2_web_viewer',
            name='ros2_web_viewer',
            output='screen',
            parameters=[ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)],
        ),

        Node(
            package='ros2_web_viewer',
            executable='restart_service_node',
            name='restart_service_node',
            output='screen',
            on_exit=Shutdown(reason='restart_service_node exited'),
        ),
    ])
