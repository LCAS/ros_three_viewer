"""ROS2 node for ros2_web_viewer.

Subscribes to standard robot topics and bridges them to a WebSocket
stream consumed by the Three.js frontend.

Topics subscribed (all configurable via ROS2 parameters):
  /robot_description      std_msgs/String                  → URDF cached, served via HTTP
  /joint_states           sensor_msgs/JointState            → JSON message type 'joint_states'
  /tf                     tf2_msgs/TFMessage                → JSON message type 'tf'
  /tf_static              tf2_msgs/TFMessage                → JSON message type 'tf' (static=true)
  <image_topics>          sensor_msgs/Image                 → JSON message type 'image' (JPEG base64)
  <pointcloud_topics>     sensor_msgs/PointCloud2           → JSON message type 'pointcloud' (binary b64)
  <marker_array_topics>   visualization_msgs/MarkerArray    → JSON message type 'marker_array'

WebSocket message format  (all JSON):
  { type: 'joint_states', name: [...], position: [...] }
  { type: 'tf', static: bool, fixed_frame: str,
    transforms: [{parent,child,tx,ty,tz,rx,ry,rz,rw},...] }
  { type: 'image',      topic, data: 'data:image/jpeg;base64,...' }
  { type: 'pointcloud', topic, frame_id, count, data: '<base64 packed float32>' }
     data layout: N × [x y z r g b] each a float32 (24 bytes/point)
  { type: 'html_panel', topic, data: '<html string>' }
  { type: 'marker_array', topic, markers: [{ns, id, type, action, frame_id,
     px, py, pz, rx, ry, rz, rw, sx, sy, sz, r, g, b, a,
     text, mesh_resource, points: [[x,y,z],...], colors: [[r,g,b,a],...]},...] }
"""

import base64
import json
import logging
import math
import os
import threading
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from .pc2_utils import decode_pointcloud2
from .server import ViewerServer

log = logging.getLogger('ros2_web_viewer.node')
log.setLevel(logging.INFO)
log.addHandler(logging.StreamHandler())

# Latching QoS for robot_description / tf_static
_LATCHING_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------

def _find_web_dir() -> str:
    """Locate the web/ directory whether running installed or from source."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ros2_web_viewer')
        candidate = os.path.join(share, 'web')
        if os.path.isdir(candidate):
            log.info(f'Web assets found in package share directory: {candidate}')
            return candidate
    except Exception:
        log.warning('ros2_web_viewer package not found; attempting to locate web assets from source tree')
        pass
    # Running from source tree
    log.warning('ros2_web_viewer running from source; web assets may not be found')
    return str(Path(__file__).parent.parent / 'web')


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class WebViewerNode(Node):

    def __init__(self, server: ViewerServer):
        super().__init__('ros2_web_viewer')
        self._server = server
        self._urdf: str | None = None
        self._tf_cache_lock = threading.Lock()
        self._tf_static_cache: dict[str, dict] = {}
        self._tf_dynamic_cache: dict[str, dict] = {}
        self._marker_cache_lock = threading.Lock()
        self._marker_cache_by_topic: dict[str, dict[str, dict]] = {}

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('image_topics', ['/camera/image_raw'])
        self.declare_parameter('pointcloud_topics', ['/points'])
        self.declare_parameter('pointcloud_max_points', 8000)
        self.declare_parameter('image_jpeg_quality', 65)
        self.declare_parameter('html_panel_topic', '/viewer_panel_html')
        self.declare_parameter('fixed_frame', 'base_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('marker_array_topics', ['/markers'])
        self._fixed_frame = self._resolve_fixed_frame()
        self._server.set_client_init_messages_getter(self._get_ws_init_messages)

        # ── Core subscriptions ───────────────────────────────────────────
        self.create_subscription(
            String, '/robot_description', self._on_urdf, _LATCHING_QOS)

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 20)

        self.create_subscription(
            TFMessage, '/tf', lambda m: self._on_tf(m, False), 50)

        self.create_subscription(
            TFMessage, '/tf_static', lambda m: self._on_tf(m, True), _LATCHING_QOS)

        # ── Dynamic topic subscriptions ──────────────────────────────────
        for topic in self.get_parameter('image_topics').value:
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._on_image(msg, t), 5)
            self.get_logger().info(f'Subscribed to image topic: {topic}')

        for topic in self.get_parameter('pointcloud_topics').value:
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, t=topic: self._on_pointcloud(msg, t), 2)
            self.get_logger().info(f'Subscribed to point cloud topic: {topic}')

        for topic in self.get_parameter('marker_array_topics').value:
            self.create_subscription(
                MarkerArray, topic,
                lambda msg, t=topic: self._on_marker_array(msg, t), 5)
            self.get_logger().info(f'Subscribed to marker_array topic: {topic}')

        html_panel_topic = str(self.get_parameter('html_panel_topic').value or '').strip()
        if html_panel_topic:
            self.create_subscription(
                String, html_panel_topic,
                lambda msg, t=html_panel_topic: self._on_html_panel(msg, t), 5)
            self.get_logger().info(f'Subscribed to html panel topic: {html_panel_topic}')

        self.get_logger().info('ros2_web_viewer node initialised')
        self.get_logger().info(f'Using fixed frame: {self._fixed_frame}')

    def _resolve_fixed_frame(self) -> str:
        """Pick fixed frame with precedence fixed_frame > target_frame > base_link.

        Leading slashes are stripped to normalise TF frame IDs.
        """
        fixed_frame = str(self.get_parameter('fixed_frame').value or '').strip()
        target_frame = str(self.get_parameter('target_frame').value or '').strip()
        selected = fixed_frame or target_frame or 'base_link'
        return selected.lstrip('/')

    # ── Accessors ────────────────────────────────────────────────────────

    @property
    def urdf(self) -> str | None:
        return self._urdf

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_urdf(self, msg: String):
        self._urdf = msg.data
        self.get_logger().info(f'robot_description received ({len(msg.data)} bytes)')

    def _on_joint_states(self, msg: JointState):
        payload = {
            'type': 'joint_states',
            'name': list(msg.name),
            'position': [float(v) for v in msg.position],
            'velocity': [float(v) for v in (msg.velocity or [])],
        }
        self._server.broadcast_threadsafe(json.dumps(payload))

    def _on_tf(self, msg: TFMessage, static: bool):
        transforms = []
        for t in msg.transforms:
            tr = t.transform.translation
            ro = t.transform.rotation
            transforms.append({
                'parent': t.header.frame_id,
                'child': t.child_frame_id,
                'tx': tr.x, 'ty': tr.y, 'tz': tr.z,
                'rx': ro.x, 'ry': ro.y, 'rz': ro.z, 'rw': ro.w,
            })
        if transforms:
            with self._tf_cache_lock:
                cache = self._tf_static_cache if static else self._tf_dynamic_cache
                for tf in transforms:
                    child = str(tf.get('child', '') or '').strip()
                    if child:
                        cache[child] = tf
            payload = {
                'type': 'tf',
                'static': static,
                'fixed_frame': self._fixed_frame,
                'transforms': transforms,
            }
            self._server.broadcast_threadsafe(json.dumps(payload))

    def _get_ws_init_messages(self) -> list[str]:
        """Return cached state that new websocket clients need immediately."""
        messages: list[str] = []
        with self._tf_cache_lock:
            static_transforms = list(self._tf_static_cache.values())
            dynamic_transforms = list(self._tf_dynamic_cache.values())
        with self._marker_cache_lock:
            marker_snapshots = {
                topic: list(marker_map.values())
                for topic, marker_map in self._marker_cache_by_topic.items()
                if marker_map
            }

        if static_transforms:
            messages.append(json.dumps({
                'type': 'tf',
                'static': True,
                'fixed_frame': self._fixed_frame,
                'transforms': static_transforms,
            }))

        if dynamic_transforms:
            messages.append(json.dumps({
                'type': 'tf',
                'static': False,
                'fixed_frame': self._fixed_frame,
                'transforms': dynamic_transforms,
            }))

        for topic, markers in marker_snapshots.items():
            messages.append(json.dumps({
                'type': 'marker_array',
                'topic': topic,
                'markers': markers,
            }))

        return messages

    def _on_image(self, msg: Image, topic: str):
        try:
            import cv2
            import numpy as np
            from cv_bridge import CvBridge
            quality = self.get_parameter('image_jpeg_quality').value
            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buf = cv2.imencode(
                '.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, quality])
            b64 = base64.b64encode(buf.tobytes()).decode('ascii')
            payload = {
                'type': 'image',
                'topic': topic,
                'data': f'data:image/jpeg;base64,{b64}',
            }
            self._server.broadcast_threadsafe(json.dumps(payload))
        except ImportError:
            self.get_logger().warning('cv_bridge / cv2 not available; image topic disabled',
                                      throttle_duration_sec=10.0)
        except Exception as exc:
            self.get_logger().warning(f'Image encode error: {exc}',
                                      throttle_duration_sec=5.0)

    def _on_pointcloud(self, msg: PointCloud2, topic: str):
        try:
            max_pts = self.get_parameter('pointcloud_max_points').value
            packed = decode_pointcloud2(msg, max_pts)
            n = len(packed) // 24  # 6 × float32 = 24 bytes per point
            if n == 0:
                return
            b64 = base64.b64encode(packed).decode('ascii')
            payload = {
                'type': 'pointcloud',
                'topic': topic,
                'frame_id': msg.header.frame_id,
                'count': n,
                'data': b64,
            }
            self._server.broadcast_threadsafe(json.dumps(payload))
        except Exception as exc:
            self.get_logger().warning(f'PointCloud2 decode error: {exc}',
                                      throttle_duration_sec=5.0)

    def _on_marker_array(self, msg: MarkerArray, topic: str):
        markers = []
        with self._marker_cache_lock:
            marker_cache = self._marker_cache_by_topic.setdefault(topic, {})

            for m in msg.markers:
                marker = {
                    'ns': m.ns,
                    'id': m.id,
                    'type': m.type,
                    'action': m.action,
                    'frame_id': m.header.frame_id,
                    'px': float(m.pose.position.x),
                    'py': float(m.pose.position.y),
                    'pz': float(m.pose.position.z),
                    'rx': float(m.pose.orientation.x),
                    'ry': float(m.pose.orientation.y),
                    'rz': float(m.pose.orientation.z),
                    'rw': float(m.pose.orientation.w),
                    'sx': float(m.scale.x),
                    'sy': float(m.scale.y),
                    'sz': float(m.scale.z),
                    'r': float(m.color.r),
                    'g': float(m.color.g),
                    'b': float(m.color.b),
                    'a': float(m.color.a),
                    'text': m.text,
                    'mesh_resource': m.mesh_resource,
                    'points': [[float(p.x), float(p.y), float(p.z)] for p in m.points],
                    'colors': [[float(c.r), float(c.g), float(c.b), float(c.a)] for c in m.colors],
                }
                markers.append(marker)

                key = f"{marker['ns']}:{marker['id']}"
                action = int(marker['action'])
                if action == 2:  # DELETE
                    marker_cache.pop(key, None)
                elif action == 3:  # DELETEALL
                    ns = str(marker['ns'] or '').strip()
                    if ns:
                        for cache_key in [k for k in marker_cache.keys() if k.startswith(f'{ns}:')]:
                            marker_cache.pop(cache_key, None)
                    else:
                        marker_cache.clear()
                else:  # ADD / MODIFY
                    marker_cache[key] = marker

            if not marker_cache:
                self._marker_cache_by_topic.pop(topic, None)

        payload = {
            'type': 'marker_array',
            'topic': topic,
            'markers': markers,
        }
        self._server.broadcast_threadsafe(json.dumps(payload))

    def _on_html_panel(self, msg: String, topic: str):
        payload = {
            'type': 'html_panel',
            'topic': topic,
            'data': msg.data,
        }
        self._server.broadcast_threadsafe(json.dumps(payload))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    # We need to pass a urdf getter before the node exists → use a closure
    _node_ref: list[WebViewerNode | None] = [None]

    def _get_urdf():
        return _node_ref[0].urdf if _node_ref[0] else None

    web_dir = _find_web_dir()
    server = ViewerServer(web_dir=web_dir, urdf_getter=_get_urdf)

    node = WebViewerNode(server)
    _node_ref[0] = node

    host = node.get_parameter_or('host', rclpy.parameter.Parameter(
        'host', value='0.0.0.0')).value
    port = node.get_parameter_or('port', rclpy.parameter.Parameter(
        'port', value=8080)).value

    # ── Start web server in a background daemon thread ───────────────────
    server_thread = threading.Thread(
        target=server.run,
        kwargs={'host': host, 'port': port},
        daemon=True,
        name='web_server',
    )
    server_thread.start()
    node.get_logger().info(f'Web viewer available at  http://{host}:{port}')

    # ── Spin ROS2 ────────────────────────────────────────────────────────
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
