# GenoM3 UAV Docker (No Gazebo)

Docker container for UAV development using GenoM3 with ROS middleware, without Gazebo simulation.

## Overview

This container provides:
- **Ubuntu 22.04 (Jammy)**
- **ROS Noetic** via robotpkg
- **GenoM3** component-based architecture
- **TeleKyB3** UAV control stack
- **Phynt** custom genom3 component

## Prerequisites

- Docker with BuildKit support
- SSH agent with keys for private GitLab/GitHub repos

## Building

```bash
# Build with SSH forwarding for private repos
DOCKER_BUILDKIT=1 docker build \
    --ssh default \
    -t genom_uav:latest \
    .

# Build without catkin_ws (faster, minimal)
DOCKER_BUILDKIT=1 docker build \
    --ssh default \
    --build-arg SETUP_CATKIN=false \
    -t genom_uav:minimal \
    .

# Build without genom_ros_bridge
DOCKER_BUILDKIT=1 docker build \
    --ssh default \
    --build-arg SETUP_ROS_GENOM_BRIDGE=false \
    -t genom_uav:no-bridge \
    .
```

## Running

```bash
# Basic run
docker run -it --rm \
    --name genom_uav \
    --network host \
    genom_uav:latest

# With X11 forwarding (for rviz, etc.)
docker run -it --rm \
    --name genom_uav \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    genom_uav:latest

# With GPU support
docker run -it --rm \
    --name genom_uav \
    --network host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    genom_uav:latest
```

## Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `SETUP_CATKIN` | `true` | Setup catkin workspace with ROS packages |
| `SETUP_ROS_GENOM_BRIDGE` | `true` | Setup genom_ros_ctl and ros_gui_bridge |

## Directory Structure

```
/root/
├── openrobots/          # robotpkg installation
├── src/robotpkg/        # robotpkg sources
├── devel/               # Development workspace
│   ├── air-phynt-genom3/
│   └── minithex-cntrl-python-scripts/
├── catkin_ws/           # ROS catkin workspace
└── scripts/             # Environment and setup scripts
```

## Environment

The container sources `/root/scripts/env.sh` on startup which sets:
- `ROBOTPKG_BASE=/root/openrobots`
- `DEVEL_BASE=/root/devel`
- ROS environment from robotpkg
- catkin_ws if available

## TeleKyB3 Components

This container includes the following genom3 components via robotpkg:
- `rotorcraft-genom3` - Low-level rotorcraft control
- `nhfc-genom3` - Near-hover flight controller
- `uavatt-genom3` - Attitude controller
- `uavpos-genom3` - Position controller
- `maneuver-genom3` - Path planning
- `pom-genom3` - Pose estimation
- `optitrack-genom3` - OptiTrack integration
- `joystick-genom3` - Joystick input
- `tk3-paparazzi` - Paparazzi autopilot integration

## Usage Examples

### Start ROS Master
```bash
roscore
```

### Start TeleKyB3 Stack
```bash
# In separate terminals or using tmux
genomixd &
rotorcraft-ros &
nhfc-ros &
pom-ros &
```

### Connect with genomix
```bash
eltclsh
% package require genomix
% genomix::connect
```

## Differences from gz_classic

This container is for **real UAV deployment** without simulation:
- No Gazebo installation
- No simulation packages (mrsim-gazebo, optitrack-gazebo)
- Minimal catkin_ws (no gazebo_ros_pkgs)
- No Gazebo environment variables
- Includes `tk3-paparazzi` for real hardware

## Troubleshooting

### SSH Key Issues During Build
Ensure your SSH agent is running and has the correct keys:
```bash
eval $(ssh-agent)
ssh-add ~/.ssh/id_rsa
```

### ROS Environment Not Set
Source the environment manually:
```bash
source /root/scripts/env.sh
```

### Missing catkin_ws
If built with `SETUP_CATKIN=false`, create manually:
```bash
source /root/openrobots/etc/ros/setup.bash
bash /root/scripts/setup_catkin_ws.sh
```
