# ROS2 Jazzy Docker Image

Docker image for ROS2 Jazzy on Ubuntu Noble 24.04 with optional Gazebo Ionic support.

## Features

- ✅ ROS2 Jazzy Desktop Full
- ✅ Ubuntu Noble 24.04
- ✅ GUI support (X11 forwarding for Turtlesim, RViz, Gazebo)
- ✅ Optional Gazebo Ionic installation
- ✅ CycloneDDS middleware
- ✅ Development tools (colcon, rosdep, vcstool)

## Build Instructions

### Without Gazebo (default):
```bash
docker build -t ros2_jazzy:latest ros2/jazzy/
```

### With Gazebo Ionic:
```bash
docker build --build-arg INSTALL_GAZEBO=yes -t ros2_jazzy:gazebo ros2/jazzy/
```

## Usage

### Basic ROS2 container:
```bash
docker run -it --rm ros2_jazzy:latest
```

### With GUI support (X11 forwarding):
```bash
# Set DISPLAY variable (WSL2)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0

# Run container
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2_jazzy:latest
```

### Test Turtlesim:
```bash
# Inside container
ros2 run turtlesim turtlesim_node
```

## Requirements

- Docker Desktop with WSL2 backend (Windows)
- VcXsrv X Server (for GUI support on Windows)

## System Requirements

- Disk space: ~4GB (without Gazebo), ~6GB (with Gazebo)
- RAM: 4GB minimum, 8GB recommended