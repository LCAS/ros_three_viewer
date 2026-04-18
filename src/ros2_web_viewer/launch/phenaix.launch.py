"""Launch PhenAIx web viewer with project topic defaults."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    viewer_launch = os.path.join(
        get_package_share_directory('ros2_web_viewer'),
        'launch',
        'viewer.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viewer_launch),
            launch_arguments={
                'image_topics': "['/camera/color/image_raw']",
                'pointcloud_topics': "['/integrated_cloud']",
                'urdf_link_whitelist': '[]',
                'urdf_link_blacklist': '[]',
            }.items(),
        ),
    ])
