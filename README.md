## ROS2 Web Viewer Workspace

This repository contains the **`ros2_web_viewer`** ROS2 package — a web-based 3D robot visualiser that bridges ROS2 topics to a browser via WebSocket using a Three.js frontend.

### Features

- Live URDF robot model rendering (box/cylinder/sphere geometries + STL meshes via `package://` URIs)
- Animated joint states from `/joint_states`
- TF2 frame transforms (`/tf` and `/tf_static`)
- Point cloud visualisation with viridis colourmap and GLSL glow shader
- Camera image stream (JPEG-compressed bridge)
- Auto-reconnecting WebSocket client
- UnrealBloom post-processing pass for a sci-fi aesthetic

### Getting Started

1. Use this repository as a template (top-right corner → **Use this template**) and specify your owner and package name.
2. Open the cloned repository in VSCode — it will prompt you to **Reopen in Container**.
3. The devcontainer will install all workspace dependencies and build the workspace automatically.

### Quick Start (without devcontainer)

```bash
# Install Python dependencies
pip3 install fastapi "uvicorn[standard]" numpy opencv-python-headless

# Install ROS2 dependencies
sudo apt install ros-${ROS_DISTRO}-cv-bridge
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select ros2_web_viewer
source install/setup.bash

# Run
ros2 launch ros2_web_viewer viewer.launch.py
```

Then open **http://localhost:8080** in a browser.

### Development

- Add ROS2 packages to the `src/` folder using `ros2 pkg create ...`
- The devcontainer runs as `root` using `ros:humble` as the base image
- Dependencies listed in `package.xml` files are installed automatically via `rosdep`
- See [`src/ros2_web_viewer/README.md`](src/ros2_web_viewer/README.md) for full package documentation

### References

- [Get Started with Dev Containers in VS Code](https://youtu.be/b1RavPr_878?si=ADepc_VocOHTXP55)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Three.js Documentation](https://threejs.org/docs/)
