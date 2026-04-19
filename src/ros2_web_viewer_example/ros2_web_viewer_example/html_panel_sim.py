# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Simulated dynamic HTML panel publisher for ros2_web_viewer_example."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HtmlPanelSimNode(Node):
    """Publishes HTML snippets on /viewer_panel_html."""

    def __init__(self):
        super().__init__('html_panel_sim')
        self.declare_parameter('publish_rate', 0.2)

        self._pub = self.create_publisher(String, '/viewer_panel_html', 5)
        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._t0 = time.time()
        self.get_logger().info('html_panel_sim started — publishing on /viewer_panel_html')

    def _publish(self):
        t = time.time() - self._t0
        msg = String()
        msg.data = (
            '<div style="font-family: Arial, sans-serif; padding: 12px; color: #1f2a18;">'
            '<h2 style="margin: 0 0 8px 0;">ROS Live Panel</h2>'
            '<p style="margin: 0 0 10px 0;">Dynamic HTML received from '
            '<code>/viewer_panel_html</code>.</p>'
            '<p style="margin: 0 0 10px 0;">'
            '<button data-trigger-service="/viewer_demo/trigger" '
            'data-trigger-timeout="2.5">Call /viewer_demo/trigger</button>'
            '</p>'
            f'<p style="margin: 0;"><strong>Uptime:</strong> {t:0.1f}s</p>'
            '</div>'
        )
        self._pub.publish(msg)

    def destroy_node(self):
        """Cancel timer and destroy the node cleanly."""
        self._timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point for html_panel_sim node."""
    rclpy.init(args=args)
    node = HtmlPanelSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
