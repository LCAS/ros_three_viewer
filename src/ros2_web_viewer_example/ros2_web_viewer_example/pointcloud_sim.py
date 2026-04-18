# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Simulated PointCloud2 publisher for ros2_web_viewer_example.

Publishes a synthetic rotating point cloud on ``/points`` at ~10 Hz.
The cloud consists of a torus and a scattered plane around the UR3e
workspace, coloured with a smooth RGB gradient.
"""

import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _build_cloud_msg(frame_id: str, stamp, points_xyz_rgb) -> PointCloud2:
    """Pack N×6 float32 array into a PointCloud2 message."""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    n = len(points_xyz_rgb)
    msg.height = 1
    msg.width = n
    msg.is_dense = True
    msg.is_bigendian = False

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.fields = fields
    msg.point_step = 16
    msg.row_step = msg.point_step * n

    buf = bytearray(msg.row_step)
    for i, (x, y, z, r, g, b) in enumerate(points_xyz_rgb):
        ri = int(max(0, min(255, r * 255)))
        gi = int(max(0, min(255, g * 255)))
        bi = int(max(0, min(255, b * 255)))
        rgb_int = (ri << 16) | (gi << 8) | bi
        rgb_f = struct.unpack('f', struct.pack('I', rgb_int))[0]
        off = i * 16
        struct.pack_into('ffff', buf, off, x, y, z, rgb_f)

    msg.data = bytes(buf)
    return msg


class PointCloudSimNode(Node):
    """Publishes a simulated point cloud around the UR3e workspace."""

    # Torus parameters (matches UR3e reach ~0.5 m)
    _TORUS_R = 0.45   # major radius
    _TORUS_r = 0.06   # minor radius
    _TORUS_N = 800    # torus sample count

    # Ground plane grid
    _PLANE_HALF = 0.8  # half-extent in metres
    _PLANE_N = 400     # number of ground plane points

    def __init__(self):
        super().__init__('pointcloud_sim')
        self.declare_parameter('frame_id', 'tool0')
        self.declare_parameter('publish_rate', 0.33)

        self._pub = self.create_publisher(PointCloud2, '/points', 5)
        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._t0 = time.time()
        self.get_logger().info('pointcloud_sim node started — publishing on /points')

    def _publish(self):
        frame_id = self.get_parameter('frame_id').value
        t = time.time() - self._t0
        points = []

        # ── Rotating torus ───────────────────────────────────────────────
        spin = t * 0.4  # rad/s
        for i in range(self._TORUS_N):
            theta = 2.0 * math.pi * i / self._TORUS_N + spin
            phi_offset = spin * 0.5
            phi = 2.0 * math.pi * i / (self._TORUS_N / 6) + phi_offset
            x = (self._TORUS_R + self._TORUS_r * math.cos(phi)) * math.cos(theta)
            y = (self._TORUS_R + self._TORUS_r * math.cos(phi)) * math.sin(theta)
            z = self._TORUS_r * math.sin(phi) + 0.5  # hover above base_link
            # HSV-like colour cycling
            hue = (theta / (2.0 * math.pi) + t * 0.1) % 1.0
            r, g, b = _hsv_to_rgb(hue, 0.9, 1.0)
            points.append((x, y, z, r, g, b))

        # ── Scattered ground plane ───────────────────────────────────────
        rng = np.random.default_rng(seed=int(t * 2) % (2**31))
        xs = rng.uniform(-self._PLANE_HALF, self._PLANE_HALF, self._PLANE_N)
        ys = rng.uniform(-self._PLANE_HALF, self._PLANE_HALF, self._PLANE_N)
        for x, y in zip(xs, ys):
            z = 0.01 * math.sin(x * 3 + t) * math.cos(y * 3 + t)
            brightness = 0.3 + 0.2 * (math.sin(x * 2 + t * 0.3) * 0.5 + 0.5)
            points.append((x, y, z, brightness, brightness * 0.8, brightness * 1.2))

        stamp = self.get_clock().now().to_msg()
        self._pub.publish(_build_cloud_msg(frame_id, stamp, points))

    def destroy_node(self):
        """Cancel timer and destroy the node cleanly."""
        self._timer.cancel()
        super().destroy_node()


def _hsv_to_rgb(h: float, s: float, v: float):
    """Simple HSV→RGB conversion, all values in [0, 1]."""
    if s == 0.0:
        return v, v, v
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i = i % 6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    return v, p, q


def main(args=None):
    """Entry point for pointcloud_sim node."""
    rclpy.init(args=args)
    node = PointCloudSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
