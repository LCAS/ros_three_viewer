# Copyright 2025 Marc Hanheide
#
# Licensed under the MIT License (the "License");
# you may not use this file except in compliance with the License.
"""Simulated MarkerArray publisher for ros2_web_viewer_example.

Publishes a ``visualization_msgs/MarkerArray`` on ``/markers`` at ~10 Hz,
demonstrating all major marker types supported by ros2_web_viewer:

  - ARROW       (type 0)  — rotating arrow using pose + scale
  - CUBE        (type 1)  — colour-flashing cube
  - SPHERE      (type 2)  — orbiting sphere
  - CYLINDER    (type 3)  — static yellow cylinder
  - TEXT        (type 9)  — animated text label
  - LINE_STRIP  (type 4)  — rainbow helix
  - SPHERE_LIST (type 7)  — animated grid of coloured spheres
  - CUBE_LIST   (type 6)  — rotating ring of coloured cubes
  - TRIANGLE_LIST (type 11) — spinning filled polygon
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


def _hsv_to_rgb(h: float, s: float, v: float):
    """Simple HSV→RGB conversion; all values in [0, 1]."""
    if s == 0.0:
        return v, v, v
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    return [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][i]


def _pt(x, y, z) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _rgba(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r)
    c.g = float(g)
    c.b = float(b)
    c.a = float(a)
    return c


class MarkerSimNode(Node):
    """Publishes a variety of animated RViz markers for viewer testing."""

    def __init__(self):
        super().__init__('marker_sim')
        self.declare_parameter('frame_id', 'tool0')
        self.declare_parameter('publish_rate', 10.0)

        self._pub = self.create_publisher(MarkerArray, '/markers', 5)
        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._t0 = time.time()
        self.get_logger().info('marker_sim started — publishing MarkerArray on /markers')

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _header(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self.get_parameter('frame_id').value
        return h

    def _base(self, ns: str, mid: int, mk_type: int) -> Marker:
        m = Marker()
        m.header = self._header()
        m.ns = ns
        m.id = mid
        m.type = mk_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0  # identity quaternion
        return m

    # ── Publish callback ─────────────────────────────────────────────────────

    def _publish(self):
        t = time.time() - self._t0
        arr = MarkerArray()
        arr.markers = self._build_markers(t)
        self._pub.publish(arr)

    def _build_markers(self, t: float) -> list:
        markers = []
        mid = 0

        # ── 1. ARROW (pose + scale) rotating around Z ────────────────────────
        m = self._base('demo', mid, Marker.ARROW)
        mid += 1
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.5
        yaw = t * 0.8
        m.pose.orientation.z = math.sin(yaw / 2)
        m.pose.orientation.w = math.cos(yaw / 2)
        m.scale.x = 0.6   # length
        m.scale.y = 0.04  # shaft diameter
        m.scale.z = 0.10  # head diameter
        m.color = _rgba(1.0, 0.3, 0.0)
        markers.append(m)

        # ── 2. CUBE (colour-flashing) ────────────────────────────────────────
        m = self._base('demo', mid, Marker.CUBE)
        mid += 1
        m.pose.position.x = 0.6
        m.pose.position.y = 0.0
        m.pose.position.z = 0.075
        m.scale.x = 0.15
        m.scale.y = 0.15
        m.scale.z = 0.15
        flash = math.sin(t * 2.0) * 0.5 + 0.5
        m.color = _rgba(flash, 0.2, 1.0 - flash)
        markers.append(m)

        # ── 3. SPHERE (orbiting) ─────────────────────────────────────────────
        m = self._base('demo', mid, Marker.SPHERE)
        mid += 1
        m.pose.position.x = 0.4 * math.cos(t * 1.2)
        m.pose.position.y = 0.4 * math.sin(t * 1.2)
        m.pose.position.z = 0.3 + 0.1 * math.sin(t * 2.5)
        m.scale.x = 0.12
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color = _rgba(0.0, 1.0, 0.5, 0.85)
        markers.append(m)

        # ── 4. CYLINDER (static) ─────────────────────────────────────────────
        m = self._base('demo', mid, Marker.CYLINDER)
        mid += 1
        m.pose.position.x = -0.5
        m.pose.position.y = 0.0
        m.pose.position.z = 0.15
        m.scale.x = 0.12  # diameter
        m.scale.y = 0.12  # diameter
        m.scale.z = 0.30  # height along Z
        m.color = _rgba(1.0, 0.8, 0.0)
        markers.append(m)

        # ── 5. TEXT_VIEWPORT_FACING ──────────────────────────────────────────
        m = self._base('demo', mid, Marker.TEXT_VIEW_FACING)
        mid += 1
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.95
        m.scale.z = 0.08  # text height in metres
        m.color = _rgba(0.9, 0.9, 0.9)
        m.text = f'ROS2 Web Viewer  t={t:.1f}s'
        markers.append(m)

        # ── 6. LINE_STRIP — rainbow helix ────────────────────────────────────
        m = self._base('demo', mid, Marker.LINE_STRIP)
        mid += 1
        m.scale.x = 0.008  # line width (hint only; WebGL ignores >1)
        m.color = _rgba(0.0, 0.8, 1.0)
        n_pts = 72
        for i in range(n_pts):
            angle = 2.0 * math.pi * i / n_pts * 4 + t * 0.6
            m.points.append(_pt(
                0.28 * math.cos(angle),
                0.28 * math.sin(angle),
                0.85 * i / n_pts,
            ))
            rc, gc, bc = _hsv_to_rgb((i / n_pts + t * 0.1) % 1.0, 1.0, 1.0)
            m.colors.append(_rgba(rc, gc, bc))
        markers.append(m)

        # ── 7. SPHERE_LIST — animated grid ───────────────────────────────────
        m = self._base('demo', mid, Marker.SPHERE_LIST)
        mid += 1
        m.scale.x = 0.04
        m.scale.y = 0.04
        m.scale.z = 0.04
        step = 5
        for ix in range(-step, step + 1):
            for iy in range(-step, step + 1):
                px = ix * 0.1
                py = iy * 0.1
                pz = 0.02 + 0.05 * math.sin(ix * 0.5 + t) * math.cos(iy * 0.5 + t * 0.7)
                m.points.append(_pt(px, py, pz))
                dist = math.sqrt(ix ** 2 + iy ** 2) / (step * math.sqrt(2))
                rc, gc, bc = _hsv_to_rgb((dist + t * 0.15) % 1.0, 0.9, 0.9)
                m.colors.append(_rgba(rc, gc, bc, 0.85))
        markers.append(m)

        # ── 8. CUBE_LIST — rotating ring ─────────────────────────────────────
        m = self._base('demo', mid, Marker.CUBE_LIST)
        mid += 1
        m.scale.x = 0.05
        m.scale.y = 0.05
        m.scale.z = 0.05
        n_cubes = 14
        for i in range(n_cubes):
            angle = 2.0 * math.pi * i / n_cubes + t * 0.4
            m.points.append(_pt(0.72 * math.cos(angle), 0.72 * math.sin(angle), 0.025))
            m.colors.append(_rgba(
                0.5 + 0.5 * math.cos(angle),
                0.5 + 0.5 * math.sin(angle),
                1.0,
            ))
        markers.append(m)

        # ── 9. TRIANGLE_LIST — spinning filled disk ───────────────────────────
        m = self._base('demo', mid, Marker.TRIANGLE_LIST)
        mid += 1
        m.pose.position.x = -0.3
        m.pose.position.y = 0.55
        m.pose.position.z = 0.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color = _rgba(0.8, 0.2, 0.9, 0.75)
        n_fan = 10
        r_outer = 0.22
        spin = t * 0.25
        for i in range(n_fan):
            a0 = 2.0 * math.pi * i / n_fan + spin
            a1 = 2.0 * math.pi * (i + 1) / n_fan + spin
            # Triangle: centre, v0, v1
            for px, py in [(0.0, 0.0),
                            (r_outer * math.cos(a0), r_outer * math.sin(a0)),
                            (r_outer * math.cos(a1), r_outer * math.sin(a1))]:
                m.points.append(_pt(px, py, 0.01))
            rc, gc, bc = _hsv_to_rgb((i / n_fan + t * 0.05) % 1.0, 0.8, 1.0)
            for _ in range(3):
                m.colors.append(_rgba(rc, gc, bc, 0.75))
        markers.append(m)

        return markers

    def destroy_node(self):
        """Cancel timer and destroy the node cleanly."""
        self._timer.cancel()
        super().destroy_node()


def main(args=None):
    """Entry point for marker_sim node."""
    rclpy.init(args=args)
    node = MarkerSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
