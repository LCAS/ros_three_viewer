# Copilot Instructions for ros_three_viewer

## Repository Overview

This repository is a ROS2 workspace containing **`ros2_web_viewer`** — a web-based 3D robot visualiser that bridges ROS2 topics to a browser via WebSocket. The frontend renders URDF robot models, live TF frames, joint states, point clouds and camera images using Three.js.

## Package: `src/ros2_web_viewer`

### Architecture

```
ROS2 System
  ├── /robot_description  ──►  HTTP GET /api/urdf  ──►  JS URDF parser  ──►  Three.js scene
  ├── /joint_states       ──┐
  ├── /tf, /tf_static     ──┤  WebSocket /ws  ──►  JSON dispatch  ──►  applyJointStates()
  ├── <image_topics>      ──┤                                           updateImage()
  └── <pointcloud_topics> ──┘                                           updatePointCloud()

FastAPI (Python, background thread)
  ├── GET  /           →  static files from web/
  ├── GET  /api/urdf   →  cached URDF string (204 until received)
  ├── GET  /mesh/<pkg>/<path>  →  ament_index mesh proxy for package:// URIs
  └── WS   /ws         →  broadcast hub to all connected browser clients
```

### Key Source Files

| File | Purpose |
|---|---|
| `ros2_web_viewer/node.py` | `WebViewerNode` — ROS2 node; subscribes to all topics and calls `ViewerServer.broadcast_threadsafe()` |
| `ros2_web_viewer/server.py` | `ViewerServer` — FastAPI app; manages WebSocket clients; runs in a daemon thread |
| `ros2_web_viewer/pc2_utils.py` | `decode_pointcloud2()` — converts `sensor_msgs/PointCloud2` to packed `N×[x,y,z,r,g,b]` float32 bytes |
| `web/app.js` | Three.js frontend; parses URDF, animates joints, renders point cloud via GLSL shader |
| `web/index.html` | Single-page app shell; uses ES module importmap for Three.js from unpkg CDN |
| `web/style.css` | Dark sci-fi aesthetic; CSS variables for colours/branding at `:root` |
| `launch/viewer.launch.py` | Standard launch file; all parameters configurable via launch arguments |
| `config/params.yaml` | Default parameter values |

### ROS2 Topics and WebSocket Message Types

**Subscribed topics** (all configurable via parameters):

| Topic | Message Type | WS `type` field |
|---|---|---|
| `/robot_description` | `std_msgs/String` | served via HTTP `/api/urdf` |
| `/joint_states` | `sensor_msgs/JointState` | `joint_states` |
| `/tf` | `tf2_msgs/TFMessage` | `tf` |
| `/tf_static` | `tf2_msgs/TFMessage` | `tf` (with `static: true`) |
| `<image_topics>` | `sensor_msgs/Image` | `image` (JPEG base64) |
| `<pointcloud_topics>` | `sensor_msgs/PointCloud2` | `pointcloud` (packed float32 base64) |

### ROS2 Parameters

| Parameter | Default | Description |
|---|---|---|
| `host` | `0.0.0.0` | Web server bind address |
| `port` | `8080` | HTTP/WS port |
| `image_topics` | `['/camera/image_raw']` | Image topics to bridge |
| `pointcloud_topics` | `['/points']` | Point cloud topics |
| `pointcloud_max_points` | `8000` | Downsampling limit per cloud |
| `image_jpeg_quality` | `65` | JPEG encode quality (1–100) |

### Dependencies

**ROS2 packages** (declared in `package.xml`):
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_msgs`, `tf2_ros`, `std_msgs`, `cv_bridge`

**Python packages** (pip-installable; also in `setup.py install_requires`):
- `fastapi`, `uvicorn[standard]`, `numpy`, `opencv-python` / `python3-opencv`

### Build & Run

```bash
# Install dependencies
pip3 install fastapi "uvicorn[standard]" numpy opencv-python-headless

# Build
cd ~/ros2_ws
colcon build --packages-select ros2_web_viewer
source install/setup.bash

# Run
ros2 launch ros2_web_viewer viewer.launch.py
# Then open http://localhost:8080
```

## Development Guidelines

### Coding Style

- Python files follow PEP 8; use f-strings for all string formatting (not `%`-style)
- Use `get_logger().warning()` (not `.warn()` which is deprecated in rclpy)
- Async code in `server.py` uses `asyncio`; ROS2 callbacks must call `broadcast_threadsafe()` (never `await` directly from a ROS2 callback thread)
- JavaScript in `web/app.js` uses ES modules; no bundler — Three.js is loaded from CDN via importmap

### Package Layout Conventions

- ROS2 Python packages use `ament_python` build type with `setup.py` + `setup.cfg`
- Web assets (`web/`) are installed to `share/ros2_web_viewer/web/` and located at runtime via `ament_index_python` or by walking up from `__file__`
- Launch files live in `launch/`, parameter files in `config/`

### Testing

- Run `colcon test --packages-select ros2_web_viewer` for ament linting tests (`ament_flake8`, `ament_pep257`, `ament_copyright`)
- The `pc2_utils.py` decoder can be unit-tested standalone (no ROS2 runtime needed) — import it directly and pass a mock message object

### Customisation

- **Branding**: edit `#logo` / `#tagline` in `web/index.html` and CSS variables in `web/style.css`
- **Bloom effect**: adjust `bloomPass` parameters in `web/app.js`
- **Add topics**: extend `image_topics` or `pointcloud_topics` parameter lists
- **Point cloud colour**: `pc2_utils.py` uses per-point RGB → intensity → height-based viridis in priority order

## CI / Workflows

| Workflow | Trigger | Purpose |
|---|---|---|
| `ros-ci.yml` | push/PR | Builds the workspace with `action-ros-ci` on Ubuntu Jammy + ROS Humble |
| `build-docker.yml` | push/tag | Builds and pushes a Docker image to GHCR |
| `dev-container.yml` | devcontainer changes | Validates the devcontainer build |
| `copilot-setup-steps.yml` | push/PR | Installs deps and builds workspace for Copilot coding agent |

Python dependencies (`fastapi`, `uvicorn`, `numpy`, `opencv-python-headless`) are installed via `pip3` in the workflow steps because they may not be present in all ROS2 rosdep databases.
