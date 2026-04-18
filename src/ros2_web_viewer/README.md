# ros2_web_viewer

A reusable ROS2 Jazzy package that serves a visually polished 3D robot viewer in the browser.
Designed for public exhibits (PhenAIx plant phenotyping platform) but fully generic.

## Features

| Feature | Detail |
|---|---|
| URDF robot model | Parsed client-side; box / cylinder / sphere geometries + async STL mesh loading |
| Joint states | Live animation from `/joint_states` |
| TF2 | `/tf` and `/tf_static` cached in a JS TF tree |
| Point cloud | Decoded from `sensor_msgs/PointCloud2`; viridis or per-point RGB; GLSL glow shader |
| Camera image | JPEG-compressed bridge from any `sensor_msgs/Image` topic |
| Post-processing | UnrealBloom pass for scanner glow effect |
| Mesh serving | `package://` URIs resolved via `ament_index_python` → served at `/mesh/<pkg>/<path>` |
| Auto-reconnect | WebSocket reconnects automatically if the backend restarts |

## Requirements

**ROS2 Jazzy** plus:

```bash
sudo apt install python3-fastapi python3-uvicorn python3-numpy python3-opencv ros-jazzy-cv-bridge
```

## Build & Install

```bash
cd ~/ros2_ws/src
# (place this package here)
cd ~/ros2_ws
colcon build --packages-select ros2_web_viewer
source install/setup.bash
```

## Run

```bash
# Minimal — uses defaults
ros2 run ros2_web_viewer ros2_web_viewer

# With launch file
ros2 launch ros2_web_viewer viewer.launch.py

# Override topics
ros2 launch ros2_web_viewer viewer.launch.py \
    image_topics:="['/realsense/color/image_raw']" \
    pointcloud_topics:="['/realsense/depth/color/points']" \
    fixed_frame:=base_link \
    port:=8080
```

Then open **http://localhost:8080** in a browser.

## Topic Summary

| Topic | Type | Notes |
|---|---|---|
| `/robot_description` | `std_msgs/String` | URDF, latching QoS |
| `/joint_states` | `sensor_msgs/JointState` | |
| `/tf` | `tf2_msgs/TFMessage` | |
| `/tf_static` | `tf2_msgs/TFMessage` | latching QoS |
| `<image_topics>` | `sensor_msgs/Image` | configurable list |
| `<pointcloud_topics>` | `sensor_msgs/PointCloud2` | configurable list |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `host` | `0.0.0.0` | Bind address |
| `port` | `8080` | HTTP/WS port |
| `image_topics` | `['/camera/image_raw']` | Image topics to bridge |
| `pointcloud_topics` | `['/points']` | Point cloud topics |
| `pointcloud_max_points` | `8000` | Cloud downsampling limit |
| `image_jpeg_quality` | `65` | JPEG quality (1–100) |
| `fixed_frame` | `base_link` | TF frame used as world/fixed frame (RViz-style) |

## Quick Test (without a real robot)

```bash
# Publish a URDF
ros2 topic pub /robot_description std_msgs/String \
  "data: '<robot name=\"test\"><link name=\"base\"><visual><geometry><box size=\"0.2 0.2 0.2\"/></geometry></visual></link></robot>'"

# Publish fake joint states
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{name: ['joint1'], position: [0.5]}"
```

## Customisation

- **Colours / theme** — edit CSS variables in `web/style.css` (`:root` block)
- **Title / branding** — edit `web/index.html` (`#logo`, `#tagline`)
- **Bloom strength** — edit `bloomPass` parameters in `web/app.js`
- **Point cloud max** — `pointcloud_max_points` ROS2 parameter
- **Add topics** — extend `image_topics` or `pointcloud_topics` parameter lists

## Architecture

```
ROS2 System
  ├── /robot_description  ──►  HTTP GET /api/urdf  ──►  JS DOMParser  ──►  Three.js scene graph
  ├── /joint_states       ──┐
  ├── /tf, /tf_static     ──┤  WebSocket /ws  ──►  JSON dispatch  ──►  applyJointStates()
  ├── /camera/image_raw   ──┤                                           updateImage()
  └── /points             ──┘                                           updatePointCloud()

FastAPI (Python, background thread)
  ├── GET  /           →  static files (web/)
  ├── GET  /api/urdf   →  cached URDF string
  ├── GET  /mesh/…     →  ament_index mesh proxy
  └── WS   /ws         →  broadcast hub
```
