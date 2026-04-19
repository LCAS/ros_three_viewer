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
| Dynamic HTML panels | Any number of HTML panel widgets updated from `std_msgs/String` topics |
| Trigger buttons | Buttons in panel HTML can call ROS `std_srvs/Trigger` services via `data-trigger-service` |
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
# Minimal — uses defaults from config/params.yaml
ros2 launch ros2_web_viewer viewer.launch.py

# With custom parameter file
ros2 launch ros2_web_viewer viewer.launch.py params_file:=path/to/custom_params.yaml

# PhenAIx-specific defaults (with phenaix_params.yaml)
ros2 launch ros2_web_viewer phenaix.launch.py

# Override parameter file at PhenAIx launch
ros2 launch ros2_web_viewer phenaix.launch.py params_file:=path/to/custom_params.yaml

# Direct node execution (uses default parameters in code)
ros2 run ros2_web_viewer ros2_web_viewer
```

Then open **http://localhost:8080** in a browser.

## Topic Summary

| Topic | Type | Notes |
|---|---|---|
| `/robot_description` | `std_msgs/String` | URDF, latching QoS |
| `/joint_states` | `sensor_msgs/JointState` | |
| `/tf` | `tf2_msgs/TFMessage` | |
| `/tf_static` | `tf2_msgs/TFMessage` | latching QoS |
| `<html_panel_topics>` | `std_msgs/String` | one or more HTML widget topics |
| `<image_topics>` | `sensor_msgs/Image` | configurable list |
| `<pointcloud_topics>` | `sensor_msgs/PointCloud2` | configurable list |

## Parameters

Parameters are configured via YAML files (primary method):
- **Default file:** [config/params.yaml](config/params.yaml) — generic defaults
- **PhenAIx defaults:** [config/phenaix_params.yaml](config/phenaix_params.yaml) — project-specific overrides
- **Custom override:** Pass `params_file:=path/to/your_params.yaml` at launch time

Alternatively, override individual parameters at runtime:
```bash
ros2 launch ros2_web_viewer viewer.launch.py params_file:=custom.yaml
ros2 run ros2_web_viewer ros2_web_viewer --ros-args -p port:=9090
```

See [config/params.yaml](config/params.yaml) for the complete, well-documented parameter reference with inline explanations.

### Quick parameter reference

For a complete, well-documented list of all parameters with explanations, see [config/params.yaml](config/params.yaml).

| Parameter | Default | Description |
|---|---|---|
| `host` | `0.0.0.0` | HTTP/WebSocket bind address |
| `port` | `8080` | HTTP/WebSocket port |
| `image_topics` | `['/camera/image_raw']` | `sensor_msgs/Image` topics to bridge |
| `pointcloud_topics` | `['/points']` | `sensor_msgs/PointCloud2` topics to render |
| `marker_array_topics` | `['/markers']` | `visualization_msgs/MarkerArray` topics to render |
| `pointcloud_max_points` | `8000` | Cloud downsampling limit |
| `image_jpeg_quality` | `65` | JPEG quality (1–100) |
| `html_panel_topics` | `['/viewer_panel_html']` | `std_msgs/String` topics for HTML panel widgets |
| `html_routes` | `{'/modular': 'examples/modular.html'}` | route→HTML file mapping (file paths relative to `web/`) |
| `fixed_frame` | `base_link` | TF frame used as world origin (RViz-style) |
| `target_frame` | `base_link` | Fallback TF frame if `fixed_frame` not set |
| `urdf_link_whitelist` | `[]` | URDF links to display (precedence over blacklist) |
| `urdf_link_blacklist` | `[]` | URDF links to hide (ignored if whitelist non-empty) |

**URDF filtering precedence:** whitelist (if non-empty) > blacklist (if non-empty) > no filtering.

`html_panel_topics` content is rendered by widgets using `data-ros-widget="html-panel"` (optionally filtered by `data-topic`). It uses the browser Sanitizer API when available (with a safe plain-text fallback).

Buttons using `<button data-trigger-service="/my_service">` call `std_srvs/Trigger` through `POST /api/trigger`.

## Quick Test (without a real robot)

```bash
# Publish a URDF
ros2 topic pub /robot_description std_msgs/String \
  "data: '<robot name=\"test\"><link name=\"base\"><visual><geometry><box size=\"0.2 0.2 0.2\"/></geometry></visual></link></robot>'"

# Publish fake joint states
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{name: ['joint1'], position: [0.5]}"

# Update right-side HTML panel
ros2 topic pub /viewer_panel_html std_msgs/String \
  "data: '<div style=\"padding:12px\"><h3>Hello from ROS</h3><p>Panel update works.</p></div>'"
```

## Customisation

- **Topics & rendering** — edit [config/params.yaml](config/params.yaml)
- **PhenAIx defaults** — edit [config/phenaix_params.yaml](config/phenaix_params.yaml)
- **Widget composition** — use `data-ros-widget="3d"` canvases and `data-ros-widget="html-panel"` containers in your HTML
- **Extra pages** — configure `html_routes` in params to map custom routes to HTML files in `web/`
- **Trigger buttons** — add `<button data-trigger-service="/my/service">` to panel HTML
- **Colours / theme** — edit CSS variables in `web/style.css` (`:root` block)
- **Title / branding** — edit `web/index.html` (`#logo`, `#tagline`)
- **Bloom strength** — edit `bloomPass` parameters in `web/app.js`

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
  ├── POST /api/trigger → std_srvs/Trigger bridge
  ├── GET  /mesh/…     →  ament_index mesh proxy
  ├── GET  <html_routes keys> → configured HTML files
  └── WS   /ws         →  broadcast hub
```
