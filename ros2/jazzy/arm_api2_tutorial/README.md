# Docker image: arm_api2 pick-and-place tutorial

This `Dockerfile` builds a container aimed solely at the **[pick-and-place tutorial](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tutorials/README.md)** for **ROS 2 Jazzy** on **Ubuntu 24.04 (noble)**. The image includes `arm_api2`, `ur_simulation_gz`, and `ur_moveit_config` (via cloned repositories), **Gazebo (gz-ionic)**, and **MoveIt 2**, without the extra “forge” stack (Nav2, SAM, WebRTC GUI, and similar).

## What is inside

| Component | Description |
|-----------|-------------|
| ROS 2 | Jazzy (`desktop`, MoveIt, MoveIt Servo, `ros2_control`, Cyclone DDS) |
| Simulation | Gazebo Ionic + `ros_gz` (sim / bridge) |
| Workspace | `/root/arms_ws` — sources under `src/`, build under `install/` |
| Repositories | `arm_api2` (branch `jazzy`), `arm_api2_msgs`, Crobotic **UR driver** and **UR GZ simulation** (branch `multiple_ur_robots`), UR **Description** (branch `jazzy`) |

Build argument:

- `ROS2_DISTRO` (default: `jazzy`) — change only if you need another distro and adjust the branches in the `Dockerfile` accordingly.

## Host prerequisites

- [Docker](https://docs.docker.com/engine/install/) (or a compatible engine)
- For **GUI** (Gazebo, RViz): X11 or Wayland with `xhost` / tunneling, or VNC — otherwise the simulator may not show a window (depends on your setup)

## Build

From the directory that contains the `Dockerfile` (e.g. this repo root):

```bash
docker build -t arm-api2-tutorial .
```

Another distro (rare):

```bash
docker build -t arm-api2-tutorial --build-arg ROS2_DISTRO=jazzy .
```

## Running the container

Interactive shell (ROS and the workspace are sourced from `.bashrc` after `bash -l` or manual `source`):

```bash
docker run -it --rm arm-api2-tutorial bash -l
```

For **Gazebo/RViz on screen** on Linux (X11, simplified):

```bash
xhost +local:root
docker run -it --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw arm-api2-tutorial bash -l
```

For more complex setups (GPU, networking, devices), adjust `docker run` to match your system.

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
