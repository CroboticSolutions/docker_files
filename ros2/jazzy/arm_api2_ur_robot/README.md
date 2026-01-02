# ROS2 Jazzy - ARM API2 + UR Robot Docker Image

Docker image for ROS2 Jazzy with ARM API2 and optional Universal Robots support, Gazebo simulation, and LeRobot integration.

## Features

- **ROS2 Jazzy Desktop Full** on Ubuntu Noble
- **ARM API2** - Robotic arm control API
- **Universal Robots** (optional) - UR robot driver and description packages
- **Gazebo Ionic** (optional) - Robot simulation
- **LeRobot** (optional) - Robot learning framework
- **MoveIt2** - Motion planning framework
- **ROS2 Control** - Hardware abstraction and controllers
- **CycloneDDS** - Default RMW implementation

## Build Arguments

All build arguments default to `"no"`. Enable features by setting them to `"yes"`:

| Argument | Default | Description |
|----------|---------|-------------|
| `USE_UR_ROBOT` | `"no"` | Installs Universal Robots packages and clones UR ROS2 repositories |
| `INSTALL_GAZEBO` | `"no"` | Installs Gazebo Ionic simulator and ROS2-Gazebo bridge |
| `LEROBOT_PY` | `"no"` | Installs LeRobot Python libraries |
| `LEROBOT_ROS` | `"no"` | Clones SO_ARM100 ROS2 workspace |

## Workspaces

**Primary workspace:** `/root/arms_ws`

Contains:
- `arm_api2` (branch: apirsic/devel)
- `arm_api2_msgs`
- `Universal_Robots_ROS2_Driver` (if `USE_UR_ROBOT="yes"`, branch: jazzy)
- `Universal_Robots_ROS2_Description` (if `USE_UR_ROBOT="yes"`, branch: jazzy)
- `Universal_Robots_ROS2_GZ_Simulation` (if `USE_UR_ROBOT="yes"`, branch: ros2)
- `ros2_so_arm100` (if `LEROBOT_ROS="yes"`)
- `feetech_ros2_driver` (if `LEROBOT_ROS="yes"`)

## Quick Start

### Build with default settings (ARM API2 only)

```bash
cd /home/toni/git/docker_files/ros2/jazzy/arm_api2_ur_robot
docker build -t ros2_armapi2:jazzy .
```

### Build with UR Robot support

```bash
docker build --build-arg USE_UR_ROBOT="yes" -t ros2_armapi2_ur:jazzy .
```

### Build with all features enabled

```bash
docker build \
  --build-arg USE_UR_ROBOT="yes" \
  --build-arg INSTALL_GAZEBO="yes" \
  --build-arg LEROBOT_PY="yes" \
  --build-arg LEROBOT_ROS="yes" \
  -t ros2_armapi2_full:jazzy .
```

## Running the Container

### Basic run

```bash
docker run -it \
  --network host \
  --privileged \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name ros2_armapi2_ur_cont \
  ros2_armapi2_ur:jazzy \
  /bin/bash
```

### Run with GPU support

```bash
docker run -it \
  --network host \
  --privileged \
  --gpus all \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name ros2_armapi2_ur_cont \
  ros2_armapi2_ur:jazzy \
  /bin/bash
```

