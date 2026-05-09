"""ROS2 node for ros2_web_viewer.

Subscribes to standard robot topics and bridges them to a WebSocket
stream consumed by the Three.js frontend.

Topics subscribed (all configurable via ROS2 parameters):
  /robot_description      std_msgs/String                  → URDF cached, served via HTTP
  /joint_states           sensor_msgs/JointState            → JSON message type 'joint_states'
  /tf                     tf2_msgs/TFMessage                → JSON message type 'tf'
  /tf_static              tf2_msgs/TFMessage                → JSON message type 'tf' (static=true)
    <viewer topics requested by 3D widgets>                  → dynamic subscriptions via /api/register_viewer_topics
  <html_panel_topics>     std_msgs/String                   → JSON message type 'html_panel' (registered by web widgets)
    <image topics requested by widgets>                      → JSON message type 'image' (JPEG base64)
    <pointcloud topics requested by widgets>                 → JSON message type 'pointcloud' (binary b64)
    <marker topics requested by widgets>                     → JSON message type 'marker_array'

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
import os
import re
import threading
import time
from pathlib import Path

import rclpy
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from .pc2_utils import decode_pointcloud2
from .server import ViewerServer

try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT = True
except ImportError:
    _HAS_AMENT = False

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
_PARAMETER_CLIENT_TIMEOUT_DEFAULT = 2.0
_PARAMETER_CLIENT_TIMEOUT_MIN = 0.1
_PARAMETER_CLIENT_TIMEOUT_MAX = 30.0
_SENSOR_THROTTLE_CHECK_PERIOD_SEC = 0.1
_ROS_PACKAGE_SUBSTITUTION_RE = re.compile(r'@([a-zA-Z0-9_]+)@')


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
        self._joint_state_cache_lock = threading.Lock()
        self._joint_state_latest_payload: dict | None = None
        self._marker_cache_lock = threading.Lock()
        self._marker_cache_by_topic: dict[str, dict[str, dict]] = {}
        self._html_panel_cache_lock = threading.Lock()
        self._html_panel_cache_by_topic: dict[str, str] = {}
        self._image_cache_lock = threading.Lock()
        self._image_latest_by_topic: dict[str, Image] = {}
        self._image_last_sent_monotonic_by_topic: dict[str, float] = {}
        self._image_subscriptions_lock = threading.Lock()
        self._image_subscriptions: dict[str, object] = {}
        self._pointcloud_cache_lock = threading.Lock()
        self._pointcloud_latest_by_topic: dict[str, PointCloud2] = {}
        self._pointcloud_last_sent_monotonic_by_topic: dict[str, float] = {}
        self._pointcloud_subscriptions_lock = threading.Lock()
        self._pointcloud_subscriptions: dict[str, object] = {}
        self._marker_array_subscriptions_lock = threading.Lock()
        self._marker_array_subscriptions: dict[str, object] = {}
        self._html_panel_subscriptions_lock = threading.Lock()
        self._html_panel_subscriptions: dict[str, object] = {}
        self._trigger_clients_lock = threading.Lock()
        self._trigger_clients: dict[str, Client] = {}
        self._parameter_clients_lock = threading.Lock()
        self._parameter_clients: dict[str, AsyncParameterClient] = {}

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('pointcloud_max_points', 8000)
        self.declare_parameter('image_jpeg_quality', 65)
        self.declare_parameter('joint_states_broadcast_hz', 1.0)
        self.declare_parameter('tf_broadcast_hz', 1.0)
        self.declare_parameter('image_broadcast_hz', 1.0)
        self.declare_parameter('pointcloud_broadcast_hz', 1.0)
        self.declare_parameter('image_topic_broadcast_hz', '{}')
        self.declare_parameter('pointcloud_topic_broadcast_hz', '{}')
        self.declare_parameter('html_routes', '{}')
        self.declare_parameter('fixed_frame', 'base_link')
        self.declare_parameter('target_frame', 'base_link')
        self._fixed_frame = self._resolve_fixed_frame()
        self._joint_states_broadcast_hz = self._resolve_positive_float_parameter('joint_states_broadcast_hz', 1.0)
        self._tf_broadcast_hz = self._resolve_positive_float_parameter('tf_broadcast_hz', 1.0)
        self._image_broadcast_hz = self._resolve_positive_float_parameter('image_broadcast_hz', 1.0)
        self._pointcloud_broadcast_hz = self._resolve_positive_float_parameter('pointcloud_broadcast_hz', 1.0)
        self._image_topic_broadcast_hz = self._resolve_topic_rate_map_parameter('image_topic_broadcast_hz')
        self._pointcloud_topic_broadcast_hz = self._resolve_topic_rate_map_parameter('pointcloud_topic_broadcast_hz')
        self._server.set_client_init_messages_getter(self._get_ws_init_messages)
        self._server.set_trigger_service_caller(self._call_trigger_service)
        self._server.set_parameter_sync_handler(self.sync_parameter_value)
        self._server.set_parameter_setter(self.set_parameter_value)
        self._server.set_viewer_topic_registrar(self.register_viewer_topics)
        self._server.set_html_panel_topic_registrar(self.register_html_panel_topic)
        self._server.set_html_routes(self._resolve_html_routes_parameter('html_routes'))

        self._joint_states_flush_timer = None
        self._tf_flush_timer = None
        if self._joint_states_broadcast_hz > 0.0:
            self._joint_states_flush_timer = self.create_timer(
                1.0 / self._joint_states_broadcast_hz,
                self._flush_joint_states,
            )
        if self._tf_broadcast_hz > 0.0:
            self._tf_flush_timer = self.create_timer(
                1.0 / self._tf_broadcast_hz,
                self._flush_dynamic_tf,
            )
        self._image_flush_timer = self.create_timer(
            _SENSOR_THROTTLE_CHECK_PERIOD_SEC,
            self._flush_images,
        )
        self._pointcloud_flush_timer = self.create_timer(
            _SENSOR_THROTTLE_CHECK_PERIOD_SEC,
            self._flush_pointclouds,
        )

        # ── Core subscriptions ───────────────────────────────────────────
        self.create_subscription(
            String, '/robot_description', self._on_urdf, _LATCHING_QOS)

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 20)

        self.create_subscription(
            TFMessage, '/tf', lambda m: self._on_tf(m, False), 50)

        self.create_subscription(
            TFMessage, '/tf_static', lambda m: self._on_tf(m, True), _LATCHING_QOS)

        # ── Dynamic topic subscriptions are registered at runtime from canvas attributes ──

        self.get_logger().info('ros2_web_viewer node initialised')
        self.get_logger().info(f'Using fixed frame: {self._fixed_frame}')
        self.get_logger().info(
            f'Throttle rates (Hz): joint_states={self._joint_states_broadcast_hz}, '
            f'tf={self._tf_broadcast_hz}, image={self._image_broadcast_hz}, '
            f'pointcloud={self._pointcloud_broadcast_hz}')
        if self._image_topic_broadcast_hz:
            self.get_logger().info(
                f'Image topic-specific throttle overrides (Hz): {self._image_topic_broadcast_hz}')
        if self._pointcloud_topic_broadcast_hz:
            self.get_logger().info(
                f'Point cloud topic-specific throttle overrides (Hz): {self._pointcloud_topic_broadcast_hz}')



    def _resolve_positive_float_parameter(self, name: str, default: float) -> float:
        """Return parameter as float. Non-positive values disable throttling."""
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return float(default)

        try:
            value = float(raw)
        except (TypeError, ValueError):
            self.get_logger().warning(
                f'Invalid value for parameter "{name}": {raw!r}. Using default {default}.')
            return float(default)
        return value

    def _resolve_topic_rate_map_parameter(self, name: str) -> dict[str, float]:
        """Resolve mapping topic->Hz from dict/JSON/python-literal parameter."""
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return {}

        parsed: dict | None = None
        if isinstance(raw, dict):
            parsed = raw
        elif isinstance(raw, str):
            raw_str = raw.strip()
            if not raw_str:
                return {}
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
                    parsed = None

        if parsed is None:
            self.get_logger().warning(
                f'Invalid topic rate map in parameter "{name}": {raw!r}. Ignoring overrides.')
            return {}

        rates: dict[str, float] = {}
        for topic, value in parsed.items():
            topic_name = str(topic or '').strip()
            if not topic_name or not self._is_valid_topic_name(topic_name):
                continue
            try:
                rates[topic_name] = float(value)
            except (TypeError, ValueError):
                continue
        return rates

    @staticmethod
    def _is_due(now: float, last_sent: float | None, hz: float) -> bool:
        if hz <= 0.0:
            return True
        if last_sent is None:
            return True
        return (now - last_sent) >= (1.0 / hz)

    def _resolve_html_routes_parameter(self, name: str) -> dict[str, str]:
        """Return route->path mapping from a dictionary-like parameter value."""
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            self.get_logger().warning(
                f'HTML routes parameter "{name}" is not yet initialized; defaulting to / -> index.html')
            return {'/': 'index.html'}

        parsed: dict | None = None
        if isinstance(raw, dict):
            parsed = raw
            self.get_logger().info(f'Using HTML routes from parameter "{name}" (dict with {len(parsed)} entries)')
        elif isinstance(raw, str):
            raw_str = raw.strip()
            self.get_logger().info(f'Parsing HTML routes from parameter "{name}" (string with length {len(raw_str)})')
            if raw_str:
                try:
                    candidate = json.loads(raw_str)
                    if isinstance(candidate, dict):
                        parsed = candidate
                        self.get_logger().info(f'Using HTML routes from parameter "{name}" (JSON string with {len(parsed)} entries)')
                    else:
                        self.get_logger().warning(
                            f'Invalid html_routes value "{raw_str}" (expected JSON dict string)')
                except json.JSONDecodeError:
                    self.get_logger().warning(
                        f'Failed to parse html_routes parameter "{name}" as JSON; trying Python literal_eval fallback')
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
            self.get_logger().info('No html_routes configured; defaulting to / -> index.html')
            return {'/': 'index.html'}

        routes: dict[str, str] = {}
        for route, path in parsed.items():
            route_str = str(route or '').strip()
            path_str = str(path or '').strip()
            if not route_str or not path_str:
                continue
            if not route_str.startswith('/'):
                route_str = f'/{route_str}'
            resolved_path = self._expand_ros_package_substitutions(path_str)
            if not resolved_path:
                self.get_logger().warning(
                    f'Skipping html route "{route_str}": could not resolve path "{path_str}"')
                continue
            routes[route_str] = resolved_path
        if not routes:
            self.get_logger().info('Resolved html_routes is empty; defaulting to / -> index.html')
            return {'/': 'index.html'}
        return routes

    def _expand_ros_package_substitutions(self, path_str: str) -> str:
        """Resolve ROS package substitutions like @package_name@."""

        def _replace(match: re.Match[str]) -> str:
            pkg_name = str(match.group(1) or '').strip()
            if not pkg_name:
                raise ValueError('empty package substitution')
            if not _HAS_AMENT:
                raise RuntimeError('ament index is not available in this environment')
            return get_package_share_directory(pkg_name)

        try:
            return _ROS_PACKAGE_SUBSTITUTION_RE.sub(_replace, path_str)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to resolve package substitution in html route path "{path_str}": {exc}')
            return ''

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

    def _is_valid_node_name(self, node_name: str) -> bool:
        return bool(re.fullmatch(r'/([A-Za-z0-9_]+(/[A-Za-z0-9_]+)*)', node_name))

    def _is_valid_parameter_name(self, parameter_name: str) -> bool:
        return bool(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*', parameter_name))

    @staticmethod
    def _normalize_parameter_type(type_name: str) -> str:
        normalized = str(type_name or '').strip().lower()
        if normalized in ('bool', 'boolean'):
            return 'bool'
        if normalized in ('int', 'integer'):
            return 'integer'
        if normalized in ('float', 'double'):
            return 'double'
        if normalized in ('str', 'string'):
            return 'string'
        return ''

    @staticmethod
    def _parameter_type_to_ros(type_name: str) -> tuple[Parameter.Type, int] | None:
        if type_name == 'bool':
            return (Parameter.Type.BOOL, ParameterType.PARAMETER_BOOL)
        if type_name == 'integer':
            return (Parameter.Type.INTEGER, ParameterType.PARAMETER_INTEGER)
        if type_name == 'double':
            return (Parameter.Type.DOUBLE, ParameterType.PARAMETER_DOUBLE)
        if type_name == 'string':
            return (Parameter.Type.STRING, ParameterType.PARAMETER_STRING)
        return None

    def _coerce_parameter_value(self, value, type_name: str):
        if type_name == 'bool':
            if isinstance(value, bool):
                return value
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if value in (0, 1):
                    return bool(value)
                raise ValueError('Expected bool-like numeric value 0 or 1')
            if isinstance(value, str):
                normalized = value.strip().lower()
                if normalized in ('1', 'true', 'yes', 'on'):
                    return True
                if normalized in ('0', 'false', 'no', 'off'):
                    return False
            raise ValueError('Expected bool value')

        if type_name == 'integer':
            if isinstance(value, bool):
                return int(value)
            if isinstance(value, int):
                return value
            if isinstance(value, float):
                if value.is_integer():
                    return int(value)
                raise ValueError('Expected integer value')
            if isinstance(value, str):
                raw = value.strip()
                try:
                    return int(raw, 10)
                except ValueError:
                    as_float = float(raw)
                    if as_float.is_integer():
                        return int(as_float)
            raise ValueError('Expected integer value')

        if type_name == 'double':
            if isinstance(value, bool):
                return float(value)
            return float(value)

        if type_name == 'string':
            return str(value)

        raise ValueError(f'Unsupported parameter type "{type_name}"')

    @staticmethod
    def _parameter_value_to_python(value_msg):
        param_type = int(value_msg.type)
        if param_type == ParameterType.PARAMETER_BOOL:
            return bool(value_msg.bool_value)
        if param_type == ParameterType.PARAMETER_INTEGER:
            return int(value_msg.integer_value)
        if param_type == ParameterType.PARAMETER_DOUBLE:
            return float(value_msg.double_value)
        if param_type == ParameterType.PARAMETER_STRING:
            return str(value_msg.string_value)
        return None

    @staticmethod
    def _wait_for_future_result(future, timeout_sec: float):
        done = threading.Event()
        result = {'value': None, 'error': None}

        def _on_done(fut):
            try:
                result['value'] = fut.result()
            except Exception as exc:
                result['error'] = exc
            finally:
                done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout_sec):
            raise TimeoutError(f'Timed out after {timeout_sec:.2f}s')
        if result['error'] is not None:
            raise result['error']
        return result['value']

    def _get_parameter_client(self, node_name: str) -> AsyncParameterClient:
        with self._parameter_clients_lock:
            client = self._parameter_clients.get(node_name)
            if client is None:
                client = AsyncParameterClient(self, node_name)
                self._parameter_clients[node_name] = client
            return client

    @staticmethod
    def _parse_parameter_timeout(payload: dict) -> float:
        timeout_raw = payload.get('timeout_sec', _PARAMETER_CLIENT_TIMEOUT_DEFAULT)
        try:
            timeout_sec = float(timeout_raw)
        except (TypeError, ValueError):
            timeout_sec = _PARAMETER_CLIENT_TIMEOUT_DEFAULT
        return max(_PARAMETER_CLIENT_TIMEOUT_MIN, min(timeout_sec, _PARAMETER_CLIENT_TIMEOUT_MAX))

    def _set_remote_parameter(
        self,
        client: AsyncParameterClient,
        node_name: str,
        parameter_name: str,
        type_name: str,
        raw_value,
        timeout_sec: float,
    ):
        type_info = self._parameter_type_to_ros(type_name)
        if type_info is None:
            raise ValueError(f'Unsupported parameter type "{type_name}"')
        parameter_type, _ = type_info
        typed_value = self._coerce_parameter_value(raw_value, type_name)
        parameter = Parameter(name=parameter_name, type_=parameter_type, value=typed_value)
        response = self._wait_for_future_result(
            client.set_parameters([parameter]),
            timeout_sec,
        )
        if not response.results:
            raise RuntimeError(f'Failed to set "{parameter_name}" on "{node_name}"')
        result = response.results[0]
        if not result.successful:
            reason = str(result.reason or 'unknown reason')
            raise RuntimeError(f'Failed to set "{parameter_name}" on "{node_name}": {reason}')
        return typed_value

    def sync_parameter_value(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            return {'ok': False, 'error': 'Invalid payload'}

        node_name = str(payload.get('node', '')).strip()
        parameter_name = str(payload.get('name', '')).strip()
        type_name = self._normalize_parameter_type(payload.get('type', ''))
        default_raw = payload.get('default_value', '')
        timeout_sec = self._parse_parameter_timeout(payload)

        if not self._is_valid_node_name(node_name):
            return {'ok': False, 'error': f'Invalid node name "{node_name}"'}
        if not self._is_valid_parameter_name(parameter_name):
            return {'ok': False, 'error': f'Invalid parameter name "{parameter_name}"'}
        if not self._parameter_type_to_ros(type_name):
            return {'ok': False, 'error': f'Unsupported parameter type "{type_name}"'}

        client = self._get_parameter_client(node_name)
        if not client.wait_for_services(timeout_sec=timeout_sec):
            self.get_logger().warning(
                f'Parameter client unavailable for node "{node_name}" while syncing "{parameter_name}"',
            )
            return {
                'ok': False,
                'error': f'Parameter services for "{node_name}" are unavailable',
                'unavailable': True,
            }

        try:
            response = self._wait_for_future_result(
                client.get_parameters([parameter_name]),
                timeout_sec,
            )
            if not response.values:
                raise RuntimeError(f'Node "{node_name}" returned no value for "{parameter_name}"')
            value_msg = response.values[0]
            value_type = int(value_msg.type)

            if value_type == ParameterType.PARAMETER_NOT_SET:
                typed_default = self._set_remote_parameter(
                    client,
                    node_name,
                    parameter_name,
                    type_name,
                    default_raw,
                    timeout_sec,
                )
                return {
                    'ok': True,
                    'node': node_name,
                    'name': parameter_name,
                    'type': type_name,
                    'value': typed_default,
                    'initialized_from_default': True,
                }

            current_value = self._parameter_value_to_python(value_msg)
            try:
                normalized_value = self._coerce_parameter_value(current_value, type_name)
            except (TypeError, ValueError) as exc:
                return {
                    'ok': False,
                    'error': f'Parameter "{parameter_name}" type mismatch on "{node_name}": {exc}',
                }

            if normalized_value != current_value:
                normalized_value = self._set_remote_parameter(
                    client,
                    node_name,
                    parameter_name,
                    type_name,
                    normalized_value,
                    timeout_sec,
                )

            return {
                'ok': True,
                'node': node_name,
                'name': parameter_name,
                'type': type_name,
                'value': normalized_value,
                'initialized_from_default': False,
            }
        except TimeoutError:
            return {
                'ok': False,
                'error': f'Timed out while syncing parameter "{parameter_name}" on "{node_name}"',
                'unavailable': True,
            }
        except (TypeError, ValueError) as exc:
            return {'ok': False, 'error': f'Invalid value for "{parameter_name}": {exc}'}
        except Exception as exc:
            return {'ok': False, 'error': f'Failed to sync "{parameter_name}" on "{node_name}": {exc}'}

    def set_parameter_value(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            return {'ok': False, 'error': 'Invalid payload'}

        node_name = str(payload.get('node', '')).strip()
        parameter_name = str(payload.get('name', '')).strip()
        type_name = self._normalize_parameter_type(payload.get('type', ''))
        value_raw = payload.get('value', '')
        timeout_sec = self._parse_parameter_timeout(payload)

        if not self._is_valid_node_name(node_name):
            return {'ok': False, 'error': f'Invalid node name "{node_name}"'}
        if not self._is_valid_parameter_name(parameter_name):
            return {'ok': False, 'error': f'Invalid parameter name "{parameter_name}"'}
        if not self._parameter_type_to_ros(type_name):
            return {'ok': False, 'error': f'Unsupported parameter type "{type_name}"'}

        client = self._get_parameter_client(node_name)
        if not client.wait_for_services(timeout_sec=timeout_sec):
            self.get_logger().warning(
                f'Parameter client unavailable for node "{node_name}" while setting "{parameter_name}"',
            )
            return {
                'ok': False,
                'error': f'Parameter services for "{node_name}" are unavailable',
                'unavailable': True,
            }

        try:
            typed_value = self._set_remote_parameter(
                client,
                node_name,
                parameter_name,
                type_name,
                value_raw,
                timeout_sec,
            )
            return {
                'ok': True,
                'node': node_name,
                'name': parameter_name,
                'type': type_name,
                'value': typed_value,
            }
        except TimeoutError:
            return {
                'ok': False,
                'error': f'Timed out while setting parameter "{parameter_name}" on "{node_name}"',
                'unavailable': True,
            }
        except (TypeError, ValueError) as exc:
            return {'ok': False, 'error': f'Invalid value for "{parameter_name}": {exc}'}
        except Exception as exc:
            return {'ok': False, 'error': f'Failed to set "{parameter_name}" on "{node_name}": {exc}'}

    def register_html_panel_topic(self, topic_name: str) -> dict:
        topic = str(topic_name or '').strip()
        if not topic:
            return {'ok': False, 'error': 'Missing topic name'}
        if not re.fullmatch(r'/([-A-Za-z0-9_]+(/[-A-Za-z0-9_]+)*)', topic):
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

    def _is_valid_topic_name(self, topic: str) -> bool:
        return bool(re.fullmatch(r'/([-A-Za-z0-9_]+(/[-A-Za-z0-9_]+)*)', topic))

    def _normalize_topic_list(self, value) -> list[str]:
        topics: list[str] = []
        if isinstance(value, (list, tuple, set)):
            items = [str(v).strip() for v in value]
        elif isinstance(value, str):
            items = [part.strip() for part in re.split(r'[\s,]+', value)]
        else:
            items = [str(value).strip()] if value is not None else []

        seen: set[str] = set()
        for item in items:
            if not item or item in seen:
                continue
            seen.add(item)
            topics.append(item)
        return topics

    def _ensure_image_subscription(self, topic: str) -> bool:
        with self._image_subscriptions_lock:
            if topic in self._image_subscriptions:
                return False
            sub = self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self._on_image(msg, t), 5,
            )
            self._image_subscriptions[topic] = sub
        self.get_logger().info(f'Subscribed to image topic: {topic}')
        return True

    def _ensure_pointcloud_subscription(self, topic: str) -> bool:
        with self._pointcloud_subscriptions_lock:
            if topic in self._pointcloud_subscriptions:
                return False
            sub = self.create_subscription(
                PointCloud2, topic,
                lambda msg, t=topic: self._on_pointcloud(msg, t), 2,
            )
            self._pointcloud_subscriptions[topic] = sub
        self.get_logger().info(f'Subscribed to point cloud topic: {topic}')
        return True

    def _ensure_marker_array_subscription(self, topic: str) -> bool:
        with self._marker_array_subscriptions_lock:
            if topic in self._marker_array_subscriptions:
                return False
            sub = self.create_subscription(
                MarkerArray, topic,
                lambda msg, t=topic: self._on_marker_array(msg, t), 5,
            )
            self._marker_array_subscriptions[topic] = sub
        self.get_logger().info(f'Subscribed to marker_array topic: {topic}')
        return True

    def register_viewer_topics(self, payload: dict) -> dict:
        if not isinstance(payload, dict):
            return {'ok': False, 'error': 'Invalid payload'}

        image_topics_list = self._normalize_topic_list(payload.get('image', []))
        pointcloud_topics_list = self._normalize_topic_list(payload.get('pointcloud', []))
        marker_topics_list = self._normalize_topic_list(payload.get('markers', []))

        invalid_topics = [
            topic for topic in (image_topics_list + pointcloud_topics_list + marker_topics_list)
            if not self._is_valid_topic_name(topic)
        ]
        if invalid_topics:
            return {
                'ok': False,
                'error': 'Invalid topic name',
                'invalid_topics': invalid_topics,
            }

        registered = {
            'image': [],
            'pointcloud': [],
            'markers': [],
        }

        for topic in image_topics_list:
            if self._ensure_image_subscription(topic):
                registered['image'].append(topic)

        for topic in pointcloud_topics_list:
            if self._ensure_pointcloud_subscription(topic):
                registered['pointcloud'].append(topic)

        for topic in marker_topics_list:
            if self._ensure_marker_array_subscription(topic):
                registered['markers'].append(topic)

        return {
            'ok': True,
            'registered': registered,
            'requested': {
                'image': image_topics_list,
                'pointcloud': pointcloud_topics_list,
                'markers': marker_topics_list,
            },
        }

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
        with self._joint_state_cache_lock:
            self._joint_state_latest_payload = payload

        if self._joint_states_broadcast_hz <= 0.0:
            self._server.broadcast_threadsafe(json.dumps(payload))

    def _flush_joint_states(self):
        with self._joint_state_cache_lock:
            payload = self._joint_state_latest_payload
        if payload is None:
            return
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
            if static or self._tf_broadcast_hz <= 0.0:
                payload = {
                    'type': 'tf',
                    'static': static,
                    'fixed_frame': self._fixed_frame,
                    'transforms': transforms,
                }
                self._server.broadcast_threadsafe(json.dumps(payload))

    def _flush_dynamic_tf(self):
        with self._tf_cache_lock:
            dynamic_transforms = list(self._tf_dynamic_cache.values())
        if not dynamic_transforms:
            return
        payload = {
            'type': 'tf',
            'static': False,
            'fixed_frame': self._fixed_frame,
            'transforms': dynamic_transforms,
        }
        self._server.broadcast_threadsafe(json.dumps(payload))

    def _get_ws_init_messages(self) -> list[str]:
        """Return cached state that new websocket clients need immediately."""
        messages: list[str] = []
        with self._tf_cache_lock:
            static_transforms = list(self._tf_static_cache.values())
            dynamic_transforms = list(self._tf_dynamic_cache.values())
        with self._joint_state_cache_lock:
            joint_states = dict(self._joint_state_latest_payload) if self._joint_state_latest_payload else None
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

        if joint_states:
            messages.append(json.dumps(joint_states))

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
        effective_hz = self._image_topic_broadcast_hz.get(topic, self._image_broadcast_hz)
        if effective_hz <= 0.0:
            self._broadcast_image_message(msg, topic)
            return

        with self._image_cache_lock:
            self._image_latest_by_topic[topic] = msg

    def _flush_images(self):
        now_monotonic = time.monotonic()
        due_messages: list[tuple[str, Image]] = []
        with self._image_cache_lock:
            for topic, msg in list(self._image_latest_by_topic.items()):
                hz = self._image_topic_broadcast_hz.get(topic, self._image_broadcast_hz)
                last_sent = self._image_last_sent_monotonic_by_topic.get(topic)
                if not self._is_due(now_monotonic, last_sent, hz):
                    continue
                due_messages.append((topic, msg))
                self._image_last_sent_monotonic_by_topic[topic] = now_monotonic
                self._image_latest_by_topic.pop(topic, None)

        for topic, msg in due_messages:
            self._broadcast_image_message(msg, topic)

    def _broadcast_image_message(self, msg: Image, topic: str):
        try:
            import cv2
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
        effective_hz = self._pointcloud_topic_broadcast_hz.get(topic, self._pointcloud_broadcast_hz)
        if effective_hz <= 0.0:
            self._broadcast_pointcloud_message(msg, topic)
            return

        with self._pointcloud_cache_lock:
            self._pointcloud_latest_by_topic[topic] = msg

    def _flush_pointclouds(self):
        now_monotonic = time.monotonic()
        due_messages: list[tuple[str, PointCloud2]] = []
        with self._pointcloud_cache_lock:
            for topic, msg in list(self._pointcloud_latest_by_topic.items()):
                hz = self._pointcloud_topic_broadcast_hz.get(topic, self._pointcloud_broadcast_hz)
                last_sent = self._pointcloud_last_sent_monotonic_by_topic.get(topic)
                if not self._is_due(now_monotonic, last_sent, hz):
                    continue
                due_messages.append((topic, msg))
                self._pointcloud_last_sent_monotonic_by_topic[topic] = now_monotonic
                self._pointcloud_latest_by_topic.pop(topic, None)

        for topic, msg in due_messages:
            self._broadcast_pointcloud_message(msg, topic)

    def _broadcast_pointcloud_message(self, msg: PointCloud2, topic: str):
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
