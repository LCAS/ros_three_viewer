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
| Parameter widgets | `input`/`select` controls can get/set ROS parameters via `data-ros-param-*` |
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
| `<data-topic attributes in HTML panel widgets>` | `std_msgs/String` | topics registered dynamically by frontend |
| `<data-topic attributes in image-panel widgets>` | `sensor_msgs/Image` | topics registered dynamically by frontend |
| `<data-pointcloud-topics attributes in 3D widgets>` | `sensor_msgs/PointCloud2` | topics registered dynamically by frontend |
| `<data-marker-topics attributes in 3D widgets>` | `visualization_msgs/MarkerArray` | topics registered dynamically by frontend |

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
| `pointcloud_max_points` | `8000` | Cloud downsampling limit |
| `image_jpeg_quality` | `65` | JPEG quality (1–100) |
| `html_routes` | `{'/': 'index.html', '/modular': 'examples/modular.html'}` | route→HTML file mapping. Paths can be relative to `web/`, absolute, or use `@package_name@` substitutions. |
| `fixed_frame` | `base_link` | TF frame used as world origin (RViz-style) |
| `target_frame` | `base_link` | Fallback TF frame if `fixed_frame` not set |

**URDF link filtering** (per-canvas via HTML attributes):
- `data-urdf-link-whitelist="link1 link2 ..."` — whitespace-separated list of links to show (if set, blacklist is ignored)
- `data-urdf-link-blacklist="link1 link2 ..."` — whitespace-separated list of links to hide (ignored if whitelist is set)

`data-ros-widget="html-panel"` widgets register their `data-topic` dynamically via `/api/register_html_panel_topic`. The content is sanitized with the browser Sanitizer API when available (with a safe plain-text fallback).

`data-ros-widget="image-panel"` widgets use `data-topic` for image subscriptions and are registered via `POST /api/register_viewer_topics`.

Buttons using `<button data-trigger-service="/my_service">` call `std_srvs/Trigger` through `POST /api/trigger`.

Inputs/selects using `data-ros-param-node`, `data-ros-param-name`, `data-ros-param-type`, and
`data-ros-param-default` are synced through `POST /api/parameter/sync` (default polling every 10s)
and updated via `POST /api/parameter/set` on change.

## Frontend Widget Attributes

The frontend discovers widgets and behavior from HTML `data-*` attributes.

### 3D Canvas (`data-ros-widget="3d"`)

Required:
- `data-ros-widget="3d"`

Display selection:
- `data-display="..."`
  - Tokens: `urdf|robot`, `pointcloud|point_cloud|cloud|pc`, `markers|marker`, plus `all` and `none`
  - Separators: whitespace or commas
  - Default when omitted: `urdf pointcloud markers`
- `data-show-urdf="true|false"`
- `data-show-pointcloud="true|false"`
- `data-show-markers="true|false"`
  - These booleans override `data-display` per feature.

Topic attributes (dynamic backend registration):
- Point cloud topics (first non-empty attribute wins):
  - `data-pointcloud-topics`
  - `data-topic-pointcloud`
  - `data-topic-cloud`
  - Format: whitespace/comma-separated absolute topic names (must start with `/`)
  - Default if pointcloud display is enabled and no attribute is provided: `/points`
- MarkerArray topics (first non-empty attribute wins):
  - `data-marker-array-topics`
  - `data-marker-topics`
  - `data-topic-markers`
  - `data-topic-marker`
  - Format: whitespace/comma-separated absolute topic names (must start with `/`)
  - Default if marker display is enabled and no attribute is provided: `/markers`

URDF link filtering:
- `data-urdf-link-whitelist="link1 link2 ..."`
- `data-urdf-link-blacklist="link1 link2 ..."`
  - If whitelist is set, blacklist is ignored.
  - Filtering only affects visual creation; link/joint transform hierarchy is preserved.

Camera tuning:
- `data-camera-fov` (default: `55`)
- `data-camera-near` (default: `0.001`)
- `data-camera-far` (default: `60`)
- `data-camera-position="x y z"` (default: `2.0 1.6 2.0`)
- `data-camera-look-at="x y z"` (default: `0 0.5 0`)

Orbit controls tuning:
- `data-controls-target="x y z"` (default: `0 0.4 0`)
- `data-controls-enable-damping="true|false"` (default: `true`)
- `data-controls-damping-factor` (default: `0.06`)
- `data-controls-min-distance` (default: `0.1`)
- `data-controls-max-distance` (default: `20`)
- `data-controls-auto-rotate="true|false"` (default: `true`)
- `data-controls-auto-rotate-speed` (default: `0.75`)

Rendering throttle:
- `data-fps-throttle`
  - Per-canvas render cap in Hz (minimum effective value: `1`)
  - Default: `5`

### HTML Panel (`data-ros-widget="html-panel"`)

Required:
- `data-ros-widget="html-panel"`
- `data-topic="/some_html_topic"` (published as `std_msgs/String`)

Expected internal role hooks:
- `[data-role="html-content"]` (panel content container)
- `[data-role="html-topic-label"]` (topic label element)

Behavior notes:
- HTML panel topics are registered dynamically via `POST /api/register_html_panel_topic`.
- Incoming HTML is sanitized with the browser Sanitizer API when available; otherwise plain text is shown.

### Image Panel (`data-ros-widget="image-panel"`)

Required:
- `data-ros-widget="image-panel"`
- `data-topic="/camera/..."` (image topic)

Expected internal role hooks:
- `[data-role="image-placeholder"]`
- `[data-role="image-topic-label"]`

Implementation note:
- For compatibility with older markup, the frontend also accepts legacy IDs (`#image-placeholder`, `#image-topic-label`).

### Trigger Buttons Inside HTML Panels

- `data-trigger-service="/my/service"` (required)
- `data-trigger-timeout="seconds"` (optional)
  - Default: `2.0`
  - Clamped range: `0.1` to `30.0`

### Parameter Controls (`input` / `select`)

- `data-ros-param-node="/target_node"` (required)
- `data-ros-param-name="parameter_name"` (required)
- `data-ros-param-type="string|bool|integer|double"` (required)
- `data-ros-param-default="..."` (required, used when parameter is not set yet)
- `data-ros-param-sync-sec="10"` (optional, default: `10`)

Behavior:
- Supports free-text (`input`) and drop-down (`select`) controls.
- Values are synced periodically (default every 10s).
- On sync, unset parameters are initialised to `data-ros-param-default`.
- On user change, values are set immediately and the control is refreshed with the typed value.
- If parameter services for the target node are unavailable, the control is disabled and an error text is shown.

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
- **Per-canvas data selection** — control each 3D canvas with `data-display="urdf pointcloud markers"` (tokens: `urdf|robot`, `pointcloud|cloud|pc`, `markers|marker`, plus `all`/`none`)
- **Display overrides** — use `data-show-urdf`, `data-show-pointcloud`, `data-show-markers` (boolean) to override `data-display`
- **Camera and controls tuning** — use canvas attributes such as `data-camera-*`, `data-controls-*`, and `data-fps-throttle`
- **Dynamic backend topic subscriptions** — configure canvas topic attributes and the frontend registers them via `POST /api/register_viewer_topics`
  - point cloud: `data-pointcloud-topics` / `data-topic-pointcloud` / `data-topic-cloud`
  - markers: `data-marker-array-topics` / `data-marker-topics` / `data-topic-markers` / `data-topic-marker`
  - image: `data-ros-widget="image-panel"` + `data-topic="/camera/..."`
- **Extra pages** — configure `html_routes` in params to map custom routes to HTML files in `web/` or package-resolved paths (for example, `@ros2_web_viewer_example@/web/examples/modular.html`)
- **Route-relative assets** — files co-located with a routed HTML file are served under that route prefix (for example, `/modular/style.css` for `/modular`)
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
  ├── GET  /assets/... → static files (web/)
  ├── GET  /api/urdf   →  cached URDF string
  ├── POST /api/trigger → std_srvs/Trigger bridge
  ├── POST /api/parameter/sync → typed ROS parameter get/init bridge
  ├── POST /api/parameter/set → typed ROS parameter set bridge
  ├── POST /api/register_viewer_topics → dynamic ROS topic registration from canvas attributes
  ├── GET  /mesh/…     →  ament_index mesh proxy
  ├── GET  <html_routes keys> → configured HTML files
  └── WS   /ws         →  broadcast hub
```
