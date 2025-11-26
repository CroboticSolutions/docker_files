# Docker Compose Setup

This directory contains Docker Compose configurations for running multiple containers.

## Prerequisites

1. Build the required images first:
   ```bash
   # Build genom gz_classic image
   cd ../genom/gz_classic
   docker build --ssh default -t genom_img:gz_old . --ssh default
   
   # Build ROS Noetic SLAM image
   cd ../../ros/noetic
   docker build --ssh default --build-arg BUILD_SLAM=true -t ros_noetic:latest .
   ```

2. Set up SSH agent socket:
   ```bash
   ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
   ```

## Usage

### Start all services
```bash
docker-compose up -d
```

**Note:** You may see a harmless `KeyError: 'id'` warning when starting services. This is a known docker-compose version issue and doesn't affect the containers.

### Start specific service
```bash
docker-compose up -d genom_gz_classic
docker-compose up -d ros_slam_noetic
```

### Attach to a running container
```bash
docker exec -it genom_gz_classic_cont bash
docker exec -it ros_slam_noetic_cont bash
```

### Alternative: Start without logs (avoids error message)
```bash
docker-compose up -d --no-log-prefix
# or
docker-compose start
```

### Stop all services
```bash
docker-compose down
```

### View logs
```bash
docker-compose logs -f
```

## Services

- **genom_gz_classic**: Genom with Gazebo Classic simulation environment
- **ros_slam_noetic**: ROS Noetic with SLAM packages (FAST-LIVO2, rpg_vikit)

Both services have:
- GPU support (NVIDIA)
- X11 forwarding for GUI applications
- SSH agent forwarding
- Host networking
- Access to all devices
