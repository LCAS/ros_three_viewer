"""Launch file for ros2_web_viewer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'host', default_value='0.0.0.0',
            description='Web server bind address'),

        DeclareLaunchArgument(
            'port', default_value='8080',
            description='Web server port'),

        DeclareLaunchArgument(
            'image_topics',
            default_value="['/camera/image_raw']",
            description='List of sensor_msgs/Image topics to bridge'),

        DeclareLaunchArgument(
            'pointcloud_topics',
            default_value="['/points']",
            description='List of sensor_msgs/PointCloud2 topics to bridge'),

        DeclareLaunchArgument(
            'pointcloud_max_points', default_value='8000',
            description='Maximum points per cloud (downsampled if exceeded)'),

        DeclareLaunchArgument(
            'image_jpeg_quality', default_value='65',
            description='JPEG quality for compressed image bridge (1-100)'),

        Node(
            package='ros2_web_viewer',
            executable='ros2_web_viewer',
            name='ros2_web_viewer',
            output='screen',
            parameters=[{
                'host':                   LaunchConfiguration('host'),
                'port':                   LaunchConfiguration('port'),
                'image_topics':           LaunchConfiguration('image_topics'),
                'pointcloud_topics':      LaunchConfiguration('pointcloud_topics'),
                'pointcloud_max_points':  LaunchConfiguration('pointcloud_max_points'),
                'image_jpeg_quality':     LaunchConfiguration('image_jpeg_quality'),
            }],
        ),
    ])
