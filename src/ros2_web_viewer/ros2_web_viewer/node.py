"""ROS2 node for ros2_web_viewer.

Subscribes to standard robot topics and bridges them to a WebSocket
stream consumed by the Three.js frontend.

Topics subscribed (all configurable via ROS2 parameters):
  /robot_description      std_msgs/String                  → URDF cached, served via HTTP
  /joint_states           sensor_msgs/JointState            → JSON message type 'joint_states'
  /tf                     tf2_msgs/TFMessage                → JSON message type 'tf'
  /tf_static              tf2_msgs/TFMessage                → JSON message type 'tf' (static=true)
  <html_panel_topics>     std_msgs/String                   → JSON message type 'html_panel' (registered by web widgets)
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
import ast
import json
import logging
import math
import os
import re
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
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
_TRIGGER_TIMEOUT_MIN = 0.1
_TRIGGER_TIMEOUT_MAX = 30.0


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------

def _find_web_dir() -> str:
    """Locate the web/ directory whether running installed or from source."""
    candidates = []

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ros2_web_viewer')
        candidates.append(os.path.join(share, 'web'))
    except Exception:
        log.warning('ros2_web_viewer package not found via ament index; trying source tree fallback')

    # Fallback: relative to this file (works from source and with symlink-install)
    candidates.append(str(Path(__file__).parent.parent / 'web'))

    for candidate in candidates:
        if not os.path.isdir(candidate):
            continue
        # Resolve symlinks on a probe file so that Starlette's StaticFiles
        # (which calls os.path.realpath on both directory and file paths when
        # checking for path-traversal) receives the real directory path.
        # This is required when built with `colcon --symlink-install`, where
        # files in the share directory are symlinks that resolve outside it.
        probe = os.path.join(candidate, 'index.html')
        if os.path.isfile(probe):
            real_dir = os.path.dirname(os.path.realpath(probe))
            log.info(f'Web assets found at: {real_dir}')
            return real_dir

    log.warning('ros2_web_viewer: could not locate web assets directory; serving may fail')
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
        self._html_panel_cache_lock = threading.Lock()
        self._html_panel_cache_by_topic: dict[str, str] = {}
        self._html_panel_subscriptions_lock = threading.Lock()
        self._html_panel_subscriptions: dict[str, object] = {}
        self._trigger_clients_lock = threading.Lock()
        self._trigger_clients: dict[str, Client] = {}

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('image_topics', ['/camera/image_raw'])
        self.declare_parameter('pointcloud_topics', ['/points'])
        self.declare_parameter('pointcloud_max_points', 8000)
        self.declare_parameter('image_jpeg_quality', 65)
        self.declare_parameter('html_routes', '{}')
        self.declare_parameter('fixed_frame', 'base_link')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('urdf_link_whitelist', [])
        self.declare_parameter('urdf_link_blacklist', [])
        self.declare_parameter('marker_array_topics', ['/markers'])
        self._urdf_link_whitelist = self._resolve_string_list_parameter('urdf_link_whitelist')
        self._urdf_link_blacklist = self._resolve_string_list_parameter('urdf_link_blacklist')
        self._fixed_frame = self._resolve_fixed_frame()
        self._server.set_client_init_messages_getter(self._get_ws_init_messages)
        self._server.set_trigger_service_caller(self._call_trigger_service)
        self._server.set_html_panel_topic_registrar(self.register_html_panel_topic)
        self._server.set_html_routes(self._resolve_html_routes_parameter('html_routes'))

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

        self.get_logger().info('ros2_web_viewer node initialised')
        self.get_logger().info(f'Using fixed frame: {self._fixed_frame}')
        if self._urdf_link_whitelist:
            if self._urdf_link_blacklist:
                self.get_logger().info(
                    'Both urdf_link_whitelist and urdf_link_blacklist were provided; '
                    'using whitelist and ignoring blacklist')
            self.get_logger().info(
                f'Using URDF link whitelist ({len(self._urdf_link_whitelist)}): {self._urdf_link_whitelist}')
        elif self._urdf_link_blacklist:
            self.get_logger().info(
                f'Using URDF link blacklist ({len(self._urdf_link_blacklist)}): {self._urdf_link_blacklist}')

    def _resolve_string_list_parameter(self, name: str) -> list[str]:
        """Return a normalised list[str] from a ROS parameter value.
        
        Returns empty list if parameter is not yet initialized (e.g., when loading from file).
        """
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            # Parameter not yet initialized from parameter file; use empty list default
            return []

        items: list[str] = []
        if isinstance(raw, (list, tuple, set)):
            items = [str(v).strip() for v in raw]
        elif isinstance(raw, str):
            raw_str = raw.strip()
            if raw_str.startswith('[') and raw_str.endswith(']'):
                try:
                    parsed = ast.literal_eval(raw_str)
                    if isinstance(parsed, (list, tuple, set)):
                        items = [str(v).strip() for v in parsed]
                    else:
                        items = [str(parsed).strip()]
                except (ValueError, SyntaxError):
                    # Fall back to comma-separated parsing.
                    items = [part.strip() for part in raw_str.split(',')]
            else:
                # Support comma-separated strings for convenience.
                items = [part.strip() for part in raw_str.split(',')]
        else:
            items = [str(raw).strip()] if raw is not None else []

        # Preserve order while removing blanks and duplicates.
        deduped: list[str] = []
        seen: set[str] = set()
        for item in items:
            if not item or item in seen:
                continue
            seen.add(item)
            deduped.append(item)
        return deduped

    def _resolve_html_routes_parameter(self, name: str) -> dict[str, str]:
        """Return route->path mapping from a dictionary-like parameter value."""
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return {}

        parsed: dict | None = None
        if isinstance(raw, dict):
            parsed = raw
        elif isinstance(raw, str):
            raw_str = raw.strip()
            if raw_str:
                try:
                    candidate = json.loads(raw_str)
                    if isinstance(candidate, dict):
                        parsed = candidate
                except json.JSONDecodeError:
                    try:
                        candidate = ast.literal_eval(raw_str)
                        if isinstance(candidate, dict):
                            parsed = candidate
                    except (ValueError, SyntaxError):
                        self.get_logger().warning(
                            f'Invalid html_routes value "{raw_str}" (expected JSON/python dict string)')
        elif raw:
            self.get_logger().warning(
                f'Ignoring html_routes of unsupported type: {type(raw).__name__}')

        if not parsed:
            return {}

        routes: dict[str, str] = {}
        for route, path in parsed.items():
            route_str = str(route or '').strip()
            path_str = str(path or '').strip()
            if not route_str or not path_str:
                continue
            if not route_str.startswith('/'):
                route_str = f'/{route_str}'
            routes[route_str] = path_str
        return routes

    def _filter_urdf_links(self, urdf_xml: str) -> tuple[str, int, int, int]:
        """Filter URDF links using whitelist/blacklist semantics.

        Precedence: whitelist > blacklist > no filtering.
        Returns (filtered_xml, original_link_count, kept_link_count, removed_joint_count).
        """
        whitelist = set(self._urdf_link_whitelist)
        blacklist = set(self._urdf_link_blacklist)

        if not whitelist and not blacklist:
            return urdf_xml, 0, 0, 0

        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as exc:
            self.get_logger().warning(
                f'Failed to parse URDF for link filtering; serving unfiltered URDF: {exc}')
            return urdf_xml, 0, 0, 0

        link_elements = [el for el in root.findall('link') if el.get('name')]
        if not link_elements:
            return urdf_xml, 0, 0, 0

        link_names = [str(el.get('name')) for el in link_elements]
        link_name_set = set(link_names)

        if whitelist:
            selected_links = link_name_set.intersection(whitelist)
        else:
            selected_links = link_name_set.difference(blacklist)

        for link_el in list(root.findall('link')):
            name = str(link_el.get('name') or '')
            if name and name not in selected_links:
                root.remove(link_el)

        removed_joints = 0
        for joint_el in list(root.findall('joint')):
            parent_el = joint_el.find('parent')
            child_el = joint_el.find('child')
            parent_link = str(parent_el.get('link') if parent_el is not None else '')
            child_link = str(child_el.get('link') if child_el is not None else '')

            if parent_link not in selected_links or child_link not in selected_links:
                root.remove(joint_el)
                removed_joints += 1

        filtered_xml = ET.tostring(root, encoding='unicode')
        return filtered_xml, len(link_names), len(selected_links), removed_joints

    def _resolve_fixed_frame(self) -> str:
        """Pick fixed frame with precedence fixed_frame > target_frame > base_link.

        Leading slashes are stripped to normalise TF frame IDs.
        Returns default 'base_link' if parameters are not yet initialized.
        """
        try:
            fixed_frame = str(self.get_parameter('fixed_frame').value or '').strip()
        except rclpy.exceptions.ParameterUninitializedException:
            fixed_frame = ''

        try:
            target_frame = str(self.get_parameter('target_frame').value or '').strip()
        except rclpy.exceptions.ParameterUninitializedException:
            target_frame = ''

        selected = fixed_frame or target_frame or 'base_link'
        return selected.lstrip('/')

    def _call_trigger_service(self, service_name: str, timeout_sec: float) -> dict:
        service = str(service_name or '').strip()
        timeout = max(_TRIGGER_TIMEOUT_MIN, min(float(timeout_sec or 2.0), _TRIGGER_TIMEOUT_MAX))
        if not service:
            return {'ok': False, 'error': 'Missing service name'}
        if not re.fullmatch(r'/([A-Za-z0-9_-]+(/[A-Za-z0-9_-]+)*)', service):
            return {'ok': False, 'error': f'Invalid service name "{service}"'}

        with self._trigger_clients_lock:
            client = self._trigger_clients.get(service)
            if client is None:
                client = self.create_client(Trigger, service)
                self._trigger_clients[service] = client

        if not client.wait_for_service(timeout_sec=timeout):
            return {'ok': False, 'error': f'Service "{service}" is unavailable'}

        done = threading.Event()
        result: dict = {'ok': False}

        future = client.call_async(Trigger.Request())

        def _on_done(fut):
            try:
                response = fut.result()
                success = bool(response.success)
                result['ok'] = success
                result['success'] = success
                result['message'] = str(response.message)
            except Exception as exc:
                result['error'] = f'Call failed: {exc}'
            finally:
                done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout):
            return {'ok': False, 'error': f'Service "{service}" timed out'}
        return result

    def register_html_panel_topic(self, topic_name: str) -> dict:
        topic = str(topic_name or '').strip()
        if not topic:
            return {'ok': False, 'error': 'Missing topic name'}
        if not re.fullmatch(r'/([A-Za-z0-9_-]+(/[A-Za-z0-9_-]+)*)', topic):
            return {'ok': False, 'error': 'Invalid topic name'}

        with self._html_panel_subscriptions_lock:
            if topic in self._html_panel_subscriptions:
                return {'ok': True, 'topic': topic, 'registered': False}
            sub = self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self._on_html_panel(msg, t),
                5,
            )
            self._html_panel_subscriptions[topic] = sub

        self.get_logger().info(f'Subscribed to html panel topic: {topic}')
        return {'ok': True, 'topic': topic, 'registered': True}

    # ── Accessors ────────────────────────────────────────────────────────

    @property
    def urdf(self) -> str | None:
        return self._urdf

    # ── Callbacks ────────────────────────────────────────────────────────

    def _on_urdf(self, msg: String):
        filtered_urdf, total_links, kept_links, removed_joints = self._filter_urdf_links(msg.data)
        self._urdf = filtered_urdf

        if total_links > 0:
            self.get_logger().info(
                'robot_description received '
                f'({len(msg.data)} bytes), URDF link filter kept {kept_links}/{total_links} links '
                f'and removed {removed_joints} joints')
        else:
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
        with self._html_panel_cache_lock:
            html_panel_snapshots = dict(self._html_panel_cache_by_topic)

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

        for topic, html_data in html_panel_snapshots.items():
            messages.append(json.dumps({
                'type': 'html_panel',
                'topic': topic,
                'data': html_data,
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
        with self._html_panel_cache_lock:
            self._html_panel_cache_by_topic[topic] = msg.data
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
