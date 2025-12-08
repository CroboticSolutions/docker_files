# ROS2 Jazzy Docker Image

Docker image for ROS2 Jazzy on Ubuntu Noble 24.04 (Linux) with optional Gazebo Ionic and LeRobot support.

## Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `INSTALL_GAZEBO` | `no` | Install Gazebo Ionic simulator (`yes`/`no`) |
| `LEROBOT_PY` | `no` | Install LeRobot with Miniforge conda environment (`yes`/`no`) |
| `LEROBOT_ROS` | `no` | Setup SO_ARM workspace with ros2_so_arm100 and feetech_ros2_driver (`yes`/`no`) |

## Features

- ✅ ROS2 Jazzy Desktop Full
- ✅ Ubuntu Noble 24.04
- ✅ GUI support (X11 forwarding for RViz, Gazebo, etc.)
- ✅ Optional Gazebo Ionic installation
- ✅ Optional LeRobot with conda environment
- ✅ Optional SO_ARM workspace for robot arm control
- ✅ CycloneDDS middleware
- ✅ Development tools (colcon, rosdep, vcstool)

## Top 5 Essential Commands

### 1. Build Basic ROS2 Image
```bash
docker build -t ros2_jazzy:latest .
```

### 2. Build with Gazebo Ionic
```bash
docker build --build-arg INSTALL_GAZEBO=yes -t ros2_jazzy:gazebo .
```

### 3. Build with LeRobot Python Package
```bash
docker build --build-arg LEROBOT_PY=yes -t ros2_jazzy:lerobot_py .
```

### 4. Build Complete LeRobot Stack (Python + SO_ARM ROS2 Workspace)
```bash
docker build --build-arg LEROBOT_PY=yes --build-arg LEROBOT_ROS=yes -t ros2_jazzy:lerobot_full .
```

### 5. Run Container with GUI Support
```bash
# On Linux, DISPLAY is usually :0
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ros2_jazzy:latest
```

## System Requirements

- Disk space: ~4GB (base), ~6GB (with Gazebo), ~8GB (with LeRobot)
- RAM: 4GB minimum, 8GB recommended
- Linux with X11 display server