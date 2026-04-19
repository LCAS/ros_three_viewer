"""Launch PhenAIx web viewer with project-specific topic defaults.

This launch file includes the generic ros2_web_viewer launcher with PhenAIx-specific
parameter defaults. To customize, either:
  1. Edit config/phenaix_params.yaml
  2. Override at runtime:
     ros2 launch ros2_web_viewer phenaix.launch.py params_file:=path/to/custom.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_web_viewer')
    viewer_launch = os.path.join(pkg_dir, 'launch', 'viewer.launch.py')
    phenaix_params = PathJoinSubstitution([pkg_dir, 'config', 'phenaix_params.yaml'])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viewer_launch),
            launch_arguments={
                'params_file': [phenaix_params],
            }.items(),
        ),
    ])
