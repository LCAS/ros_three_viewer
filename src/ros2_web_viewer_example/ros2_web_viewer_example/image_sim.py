# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Simulated camera image publisher for ros2_web_viewer_example.

Publishes a synthetic BGR8 image on ``/camera/image_raw`` at ~10 Hz.
The frame shows an animated sine-wave pattern with a timestamp overlay,
simulating a rudimentary streaming camera pointed at the robot workspace.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class ImageSimNode(Node):
    """Publishes a simulated camera image."""

    _WIDTH = 640
    _HEIGHT = 480

    def __init__(self):
        super().__init__('image_sim')
        self.declare_parameter('publish_rate', 10.0)

        self._pub = self.create_publisher(Image, '/camera/image_raw', 5)
        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._t0 = time.time()
        self.get_logger().info('image_sim node started — publishing on /camera/image_raw')

    def _publish(self):
        t = time.time() - self._t0

        # ── Animated background gradient ─────────────────────────────────
        xs = np.linspace(0.0, 2.0 * math.pi, self._WIDTH, dtype=np.float32)
        ys = np.linspace(0.0, 2.0 * math.pi, self._HEIGHT, dtype=np.float32)
        xx, yy = np.meshgrid(xs, ys)

        r = (np.sin(xx + t * 1.2) * 0.5 + 0.5) * 180 + 20
        g = (np.sin(yy + t * 0.8) * 0.5 + 0.5) * 180 + 20
        b = (np.sin(xx * 0.5 + yy * 0.5 - t * 0.6) * 0.5 + 0.5) * 180 + 20

        frame = np.stack([b, g, r], axis=2).astype(np.uint8)

        # ── Crosshair overlay ────────────────────────────────────────────
        cx, cy = self._WIDTH // 2, self._HEIGHT // 2
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (255, 255, 255), 1)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (255, 255, 255), 1)
        cv2.circle(frame, (cx, cy), 40, (200, 200, 200), 1)

        # ── Timestamp overlay ────────────────────────────────────────────
        label = f'ROS2 sim  t={t:.2f}s'
        cv2.putText(frame, label, (10, self._HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)

        # ── Pack into ROS2 Image message ─────────────────────────────────
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = self._HEIGHT
        msg.width = self._WIDTH
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = self._WIDTH * 3
        msg.data = frame.tobytes()

        self._pub.publish(msg)

    def destroy_node(self):
        """Cancel timer and destroy the node cleanly."""
        self._timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point for image_sim node."""
    rclpy.init(args=args)
    node = ImageSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
