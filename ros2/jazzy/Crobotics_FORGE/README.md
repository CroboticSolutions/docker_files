# Crobotics FORGE Docker Environment (ROS 2 Jazzy)

Dockerized ROS 2 Jazzy development environment for Crobotics robot stacks on Ubuntu 24.04 (`ubuntu:noble`).

The image provides:

- ROS 2 Jazzy desktop + common robotics tooling (`colcon`, `rosdep`, Nav2, MoveIt)
- Optional Universal Robots dependencies and repos
- Optional Gazebo (`gz-ionic` + ROS Gazebo bridge packages)
- Optional Crobotic FORGE stack components (WebRTC, GUI, SAM integration)
- Preconfigured shell startup (`python venv`, ROS setup files, workspace overlay)

## Directory Contents

- `Dockerfile` - image definition
- `run_docker.sh` - host-network interactive container launcher
- `ur_gui_stack.yml` - tmuxinator layout for GUI/services

## Prerequisites

- Docker installed and running
- Linux host with X11 available (for GUI apps)
- Git access to CroboticSolutions private repositories if using:
  - `CROBOTIC_FORGE=yes`
  - `USE_UR_ROBOT=yes`
- SSH agent socket available on host (`$SSH_AUTH_SOCK`)

If you use X11 forwarding, allow local Docker clients:

```bash
xhost +local:docker
```

## Build Image

From this folder:

```bash
docker build -t ros2_img:latest .
```

### Build Arguments

The `Dockerfile` supports these build args:

- `USE_UR_ROBOT` (`yes`/`no`, default `no`)
- `INSTALL_GAZEBO` (`yes`/`no`, default `no`)
- `CROBOTIC_FORGE` (`yes`/`no`, default `no`)
- `PY_VENV` (default `/root/py_global`)

Examples:

Minimal core build:

```bash
docker build -t ros2_img:latest \
  --build-arg USE_UR_ROBOT=no \
  --build-arg INSTALL_GAZEBO=no \
  --build-arg CROBOTIC_FORGE=no \
  .
```

UR + Gazebo build:

```bash
docker build -t ros2_img:latest \
  --build-arg USE_UR_ROBOT=yes \
  --build-arg INSTALL_GAZEBO=yes \
  --build-arg CROBOTIC_FORGE=no \
  .
```

Full FORGE build (requires access to private repos):

```bash
docker build -t ros2_img:latest \
  --build-arg USE_UR_ROBOT=yes \
  --build-arg INSTALL_GAZEBO=yes \
  --build-arg CROBOTIC_FORGE=yes \
  .
```

## Run Container

Use the provided launcher:

```bash
chmod +x run_docker.sh
./run_docker.sh
```

The script starts an interactive container:

- Name: `ros2_jazzy_Crobotics_FORGE_cont`
- Image: `ros2_img:latest`
- `--network host`
- `--privileged`
- Device passthrough via `/dev`
- X11 socket mount (`/tmp/.x11-unix`)
- SSH agent forwarding to `/ssh-agent`

## Inside the Container

On shell startup, `.bashrc` automatically sources:

- `/root/py_global/bin/activate`
- `/opt/ros/jazzy/setup.bash`
- `/root/arms_ws/install/setup.bash`

Workspace path:

```bash
/root/arms_ws
```

Rebuild workspace manually if needed:

```bash
cd /root/arms_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## Included Source Repositories

Always cloned:

- `CroboticSolutions/arm_api2` (`multiple_ur_robots` branch)
- `CroboticSolutions/arm_api2_msgs`

When `CROBOTIC_FORGE=yes`:

- `CroboticSolutions/aiortc_webrtc_ros`
- `CroboticSolutions/ros2_dash_gui` (`multiple_ur_robots`)
- `CroboticSolutions/ros2_robotiq_gripper` (`multiple_ur_robots`)
- `CroboticSolutions/sam_ros2` (`multiple_ur_robots`)
- Segment Anything checkpoint downloaded to:
  - `/root/arms_ws/src/segment-anything/checkpoints/sam_vit_b_01ec64.pth`

When `USE_UR_ROBOT=yes`:

- `CroboticSolutions/Universal_Robots_ROS2_Driver` (`multiple_ur_robots`)
- `CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation` (`multiple_ur_robots`)
- `UniversalRobots/Universal_Robots_ROS2_Description` (`jazzy`)

## Tmuxinator Stack

The file `ur_gui_stack.yml` is copied to:

```bash
/root/.tmuxinator/ur_gui_stack.yml
```

Run:

```bash
tmuxinator start ur_gui_stack
```

This launches multiple panes for:

- `ros2_dash_gui` bridge
- frontend dev server (`npm run dev`)
- WebRTC ROS servers (ports `8081`, `8082`)
- FastAPI backend (`uvicorn` on `127.0.0.1:8000`)
- `sam_ros2` interactive launch

## Troubleshooting

- SSH clone fails during build:
  - You are probably missing access to private repos or SSH agent forwarding.
- GUI apps do not open:
  - Check `DISPLAY`, `xhost`, and `/tmp/.x11-unix` mount.
- Container name conflict:
  - Remove old container:
    ```bash
    docker rm -f ros2_jazzy_Crobotics_FORGE_cont
    ```

## Security Note

This setup uses `--privileged`, host networking, and host device mounts to support robotics workflows. Do not use this configuration on untrusted hosts or for multi-tenant environments without hardening.
