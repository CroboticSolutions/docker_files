# DemoMotion Docker Compose

Docker Compose setup for DemoMotion project with two ROS2 Jazzy services:
- **arm_api2_ur**
- **mediapipe**

## Build Arguments

**arm_api2_ur** service is built with all build arguments set to "yes":
- `INSTALL_GAZEBO="yes"` - Installs Gazebo Ionic simulator
- `LEROBOT_PY="yes"` - Installs LeRobot Python libraries
- `LEROBOT_ROS="yes"` - Clones SO_ARM100 workspace
- `USE_UR_ROBOT="yes"` - Clones Universal Robots ROS2 driver and description packages

## How to Use

### 1. Build both images

```bash
cd /home/toni/git/docker_files/composers/demomotion
DOCKER_BUILDKIT=1 ./docker-compose-up.sh build
```

**Note:** DOCKER_BUILDKIT=1 is required because mediapipe uses SSH mount for private repositories.

### 2. Start both containers in detached mode

```bash
./docker-compose-up.sh up -d
```

### 3. Attach to the container you want

**arm_api2_ur:**
```bash
docker exec -it ros2_armapi2_ur_cont bash
```

**mediapipe:**
```bash
docker exec -it ros2_mp_cont bash
```

### 4. Start only one service

```bash
./docker-compose-up.sh up -d arm_api2_ur
# or
./docker-compose-up.sh up -d mediapipe
```

### 5. Interactive mode for one service

```bash
./docker-compose-up.sh run --rm arm_api2_ur
# or
./docker-compose-up.sh run --rm mediapipe
```

### 6. Stop all services

```bash
./docker-compose-up.sh down
```

