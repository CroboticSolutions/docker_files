# Docker Compose Setup

This directory contains Docker Compose configurations for running multiple containers.

## Prerequisites

1. Build the required images first:
   ```bash
   # Build genom gz_classic_react image
   cd ../../genom/gz_classic
   docker build --ssh default -t genom_img:gz_classic_react .

   # Build ROS SLAM Noetic image
   cd ../../ros/noetic
   docker build --ssh default --build-arg BUILD_SLAM=true -t ros_slam_img:noetic .
   ```

2. Set up SSH agent socket:
   ```bash
   ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock
   ```

## Usage

### Quick Start with Tmuxinator (Recommended)

The easiest way to start everything in one go:

1. **Setup tmuxinator** (first time only):
   ```bash
   ./setup_tmuxinator.sh
   ```

2. **Launch everything**:
   ```bash
   tmuxinator start autopilot_ndt
   ```

   This will automatically:
   - Start both Docker containers (`genom_gz_classic_react_cont` and `ros_slam_noetic_cont`)
   - Launch `roslaunch fast_livo mapping_sim.launch` in ros_slam_noetic container
   - Launch `roslaunch ros_gui_bridge bridge.launch` in genom_gz container
   - Start `minithex_demo` tmuxinator session
   - Start React GUI with `npm run dev` in minithex_react_gui directory

3. **Stop everything**:
   ```bash
   tmux kill-session -t autopilot_ndt
   ```
   The containers will automatically stop when you exit the tmuxinator session.

**Optional:** Set custom compose path:
```bash
AUTOPILOTNDT_COMPOSE_PATH=/path/to/compose/directory tmuxinator start autopilot_ndt
```

### Manual Start

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

## Note: 
Added volume for the `ssh` keys, not the best way maybe but it works.


## TODO: 
- [ ] Test and debug full autoassess_ndt.yml tmuxinator 
- [ ] Decouple sim and real robot 
- [ ] Write full instructions 
