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

        DeclareLaunchArgument(
            'html_panel_topic', default_value='/viewer_panel_html',
            description='std_msgs/String topic used to populate the right-side HTML panel'),

        DeclareLaunchArgument(
            'fixed_frame', default_value='base_link',
            description='Fixed TF frame used as world origin in the viewer'),

        DeclareLaunchArgument(
            'urdf_link_whitelist',
            default_value='[]',
            description='List of URDF link names to display; takes precedence over blacklist'),

        DeclareLaunchArgument(
            'urdf_link_blacklist',
            default_value='[]',
            description='List of URDF link names to hide when whitelist is empty'),

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
                'html_panel_topic':       LaunchConfiguration('html_panel_topic'),
                'fixed_frame':            LaunchConfiguration('fixed_frame'),
                'urdf_link_whitelist':    LaunchConfiguration('urdf_link_whitelist'),
                'urdf_link_blacklist':    LaunchConfiguration('urdf_link_blacklist'),
            }],
        ),
    ])
