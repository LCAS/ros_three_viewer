# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Launch file for ros2_web_viewer_example.

Starts:
  1. UR3e robot description via ``ur_description view_ur.launch.xml``
     (publishes /robot_description, /joint_states and /tf via joint_state_publisher)
  2. Simulated point cloud publisher (pointcloud_sim node)
  3. Simulated camera image publisher (image_sim node)
  4. ros2_web_viewer server

Open http://localhost:8080 in a browser once everything is running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the example launch description."""
    return LaunchDescription([

        # ── Launch arguments ─────────────────────────────────────────────

        DeclareLaunchArgument(
            'ur_type', default_value='ur3e',
            description='UR robot type (e.g. ur3e, ur5e, ur10e)'),

        DeclareLaunchArgument(
            'host', default_value='0.0.0.0',
            description='Web server bind address'),

        DeclareLaunchArgument(
            'port', default_value='8080',
            description='Web server port'),

        DeclareLaunchArgument(
            'enable_pointcloud_sim', default_value='false',
            description='Launch the simulated point cloud publisher'),

        DeclareLaunchArgument(
            'enable_image_sim', default_value='false',
            description='Launch the simulated camera image publisher'),

        DeclareLaunchArgument(
            'enable_marker_sim', default_value='false',
            description='Launch the simulated MarkerArray publisher'),

        DeclareLaunchArgument(
            'enable_html_panel_sim', default_value='true',
            description='Launch the simulated HTML panel String publisher'),

        DeclareLaunchArgument(
            'enable_trigger_service_sim', default_value='true',
            description='Launch a demo std_srvs/Trigger service for HTML panel buttons'),

        # ── UR3e robot description + joint_state_publisher ───────────────

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros2_web_viewer_example'),
                    'launch',
                    'ur_description.launch.py',
                ]),
            ]),
            launch_arguments={
                'ur_type': LaunchConfiguration('ur_type'),
            }.items(),
        ),

        # ── Simulated point cloud ────────────────────────────────────────

        Node(
            package='ros2_web_viewer_example',
            executable='pointcloud_sim',
            name='pointcloud_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_pointcloud_sim')),
        ),

        # ── Simulated camera image ───────────────────────────────────────

        Node(
            package='ros2_web_viewer_example',
            executable='image_sim',
            name='image_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_image_sim')),
        ),

        # ── Simulated MarkerArray ────────────────────────────────────────

        Node(
            package='ros2_web_viewer_example',
            executable='marker_sim',
            name='marker_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_marker_sim')),
        ),

        # ── Simulated dynamic HTML panel ──────────────────────────────────

        Node(
            package='ros2_web_viewer_example',
            executable='html_panel_sim',
            name='html_panel_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_html_panel_sim')),
        ),

        # ── Simulated Trigger service for panel buttons ─────────────────────

        Node(
            package='ros2_web_viewer_example',
            executable='trigger_service_sim',
            name='trigger_service_sim',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_trigger_service_sim')),
        ),

        # ── Web viewer ───────────────────────────────────────────────────

        Node(
            package='ros2_web_viewer',
            executable='ros2_web_viewer',
            name='ros2_web_viewer',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'image_topics': ['/camera/image_raw'],
                'pointcloud_topics': ['/points'],
                'marker_array_topics': ['/markers'],
                'pointcloud_max_points': 5000,
                'image_jpeg_quality': 65,
                'html_routes': "{'/modular': 'examples/modular.html'}",
            }],
        ),
    ])
