# Docker image: arm_api2 pick-and-place tutorial

This `Dockerfile` builds a container aimed solely at the **[pick-and-place tutorial](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tutorials/README.md)** for **ROS 2 Jazzy** on **Ubuntu 24.04 (noble)**. The image includes `arm_api2`, `ur_simulation_gz`, and `ur_moveit_config` (via cloned repositories), **Gazebo (gz-ionic)**, and **MoveIt 2**, without the extra “forge” stack (Nav2, SAM, WebRTC GUI, and similar).

## What is inside

| Component | Description |
|-----------|-------------|
| ROS 2 | Jazzy (`desktop`, MoveIt, MoveIt Servo, `ros2_control`, Cyclone DDS) |
| Simulation | Gazebo Ionic + `ros_gz` (sim / bridge) |
| Workspace | `/root/arms_ws` — sources under `src/`, build under `install/` |
| Repositories | `arm_api2` (branch `jazzy`), `arm_api2_msgs`, Crobotic **UR driver**, **UR GZ simulation**, and **ros2_robotiq_gripper** (branch `multiple_ur_robots`), **`tylerjw/serial`** (branch `ros2`, CMake `serial` for `robotiq_driver`), UR **Description** (branch `jazzy`) |

Build argument:

- `ROS2_DISTRO` (default: `jazzy`) — change only if you need another distro and adjust the branches in the `Dockerfile` accordingly.

## Host prerequisites

- [Docker](https://docs.docker.com/engine/install/) (or a compatible engine)
- For **GUI** (Gazebo, RViz): X11 or Wayland with `xhost` / tunneling, or VNC — otherwise the simulator may not show a window (depends on your setup)

## Build and run (recommended)

From the directory that contains `Dockerfile` and `first-run.sh`:

1. **Build** the image:

   ```bash
   docker build -t arm_api2:tutorial .
   ```

2. **Start** a container (login shell; ROS and workspace sourced via `.bashrc`):

   ```bash
   chmod +x first-run.sh   # once
   ./first-run.sh --no-build
   ```

   Or let the script **build and run** in one step (same tag `arm_api2:tutorial`):

   ```bash
   ./first-run.sh
   ```

Useful flags:

```bash
./first-run.sh --help
./first-run.sh --x11        # X11 forwarding for Gazebo / RViz (on the host: e.g. xhost +local:root)
./first-run.sh --net-host   # host network (often helps ROS 2 DDS)
```

Override the image name if needed:

```bash
IMAGE_NAME=myrepo:mytag ./first-run.sh --no-build
```

## Manual `docker run` (optional)

If you built with `docker build -t arm_api2:tutorial .`:

```bash
docker run -it --rm arm_api2:tutorial bash -l
```

For **Gazebo/RViz on screen** on Linux (X11, simplified):

```bash
xhost +local:root
docker run -it --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw arm_api2:tutorial bash -l
```

Another distro when building (rare):

```bash
docker build -t arm_api2:tutorial --build-arg ROS2_DISTRO=jazzy .
```

## Tutorial: two terminals in the container

After `bash -l` inside the container, follow the [official tutorial README](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tutorials/README.md).

**Terminal 1** — simulation + MoveIt:

```bash
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py robots_profile:=lab_gripper_one
```

**Terminal 2** — `arm_api2` interface:

```bash
ros2 launch arm_api2 moveit2_simple_iface.launch.py \
  robot_name:=ur robot_ns:=ur1 use_sim_time:=true
```

Then either the manual steps from the tutorial or the bundled helper:

```bash
ros2 run arm_api2 pick_place_sequence.py
```

The default YAML sequence usually lives under `share/arm_api2/tutorials/` in the install space; see the tutorial document for details.

## Rebuilding the workspace

If you edit `src/` inside the container:

```bash
cd /root/arms_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Notes

- The image is intentionally **narrower in scope** than a full development stack: faster builds and less surface area to maintain, assuming you follow the tutorial above.
- If `docker build` fails on `rosdep` or a missing apt package, verify that the repository branches in the `Dockerfile` still match your ROS 2 release.
