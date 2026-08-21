# Docker image: arm_api2 selftest (Jazzy)

This `Dockerfile` builds a container for the **[arm_api2 selftest](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tests/arm_api2_selftest/README.md)** on **ROS 2 Jazzy** / **Ubuntu 24.04 (noble)**. The image includes `arm_api2`, MoveIt 2, the Crobotic UR driver, and the Robotiq gripper stack. **Gazebo (gz-ionic)** and `ur_simulation_gz` are **optional** and only installed when you pass `ENABLE_GAZEBO=1`.

Sources are cloned from the **live branch heads** listed below (not pinned SHAs). A rebuild picks up whatever is currently on those branches.

## What is inside

| Component | Description |
|-----------|-------------|
| ROS 2 | Jazzy (`desktop`, MoveIt, MoveIt Servo, `ros2_control`, Cyclone DDS) |
| Simulation | Optional: Gazebo Ionic + `ros_gz` (sim / bridge) |
| Workspace | `/root/arms_ws` — sources under `src/`, build under `install/` |
| Repositories | `arm_api2` (`jazzy`), `arm_api2_msgs` (`apirsic/devel`), Crobotic **UR driver** (`multiple_ur_robots`), Crobotic **ros2_robotiq_gripper** (`multiple_ur_robots`), **`tylerjw/serial`** (`ros2`, CMake `serial` for `robotiq_driver`), UR **Description** (`jazzy`). With Gazebo: Crobotic **UR GZ simulation** (`multiple_ur_robots`) |

Build arguments:

- `ROS2_DISTRO` (default: `jazzy`) — change only if you need another distro and adjust the clone branches in the `Dockerfile` accordingly.
- `ENABLE_GAZEBO` (default: `0`) — set to `1` to install Gazebo Ionic, `ros_gz`, and clone `Universal_Robots_ROS2_GZ_Simulation`. Required for the simulated selftest path.

## Host prerequisites

- [Docker](https://docs.docker.com/engine/install/) (or a compatible engine)
- For **GUI** (Gazebo, RViz): X11 or Wayland with `xhost` / tunneling, or VNC — otherwise the simulator may not show a window (depends on your setup)

## Build and run

From the directory that contains `Dockerfile` and `first-run.sh`:

1. **Build** the image.

   Without Gazebo (smaller image; use a real robot or attach to an already-running sim):

   ```bash
   docker build -t arm_api2:selftest-jazzy .
   ```

   With Gazebo (needed to launch `ur_simulation_gz` inside the container):

   ```bash
   docker build -t arm_api2:selftest-jazzy --build-arg ENABLE_GAZEBO=1 .
   ```

2. **Run** the container (via `first-run.sh` in this repo):

   ```bash
   chmod +x first-run.sh   # once
   ./first-run.sh
   ```

   What `first-run.sh` does (edit the script if you want other names or flags):

   - **Image:** `arm_api2:selftest-jazzy`
   - **Container name:** `arm_api2_selftest_jazzy`
   - **`docker run`:** `-it`, `--network host`, `--privileged`, bind-mount `/dev`, bind-mount `/tmp/.x11-unix` → `/tmp/.x11-unix`, `DISPLAY`, `TERM=xterm-256color`, interactive `/bin/bash`
   - **SSH agent:** `ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock` on the host, then mount `~/.ssh/ssh_auth_sock` → `/ssh-agent` with `SSH_AUTH_SOCK=/ssh-agent` ([tmux / VS Code forwarding notes](https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/))

   Before the first run, create `~/.ssh` on the host if it does not exist. If a container with the same name is already there, remove it: `docker rm -f arm_api2_selftest_jazzy`.

   For **Gazebo / RViz** on the host you may need: `xhost +local:root`.  
   Many Linux systems expose the X11 socket as **`/tmp/.X11-unix`** (capital `X`); if the GUI does not show, align the host path in `first-run.sh` with your system.

   A **second shell** in the same container:

   ```bash
   docker exec -it arm_api2_selftest_jazzy /bin/bash
   ```

## Selftest: terminals in the container

Inside the container you get interactive `/bin/bash`; the image’s `/root/.bashrc` sources ROS Jazzy and `/root/arms_ws/install/setup.bash`, so that environment should load automatically. Full details: [selftest README](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tests/arm_api2_selftest/README.md).

**Terminal 1** — simulation + MoveIt (requires `ENABLE_GAZEBO=1`):

```bash
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py robots_profile:=lab_gripper_one
```

**Terminal 2** — `arm_api2` interface (`mode:=advanced` is required; it is the default):

```bash
ros2 launch arm_api2 moveit2_iface.launch.py \
  robot_name:=ur robot_ns:=ur1 use_sim_time:=true mode:=advanced
```

`use_sim_time:=true` matters in simulation: without it, `moveit2_iface`'s clock disagrees with Gazebo and MoveGroupInterface fails to fetch robot state.

**Terminal 3** — safe checks first, then real motion:

```bash
ros2 run arm_api2 arm_api2_selftest.py --robot-ns ur1
ros2 run arm_api2 arm_api2_selftest.py --robot-ns ur1 --move
```

## Rebuilding the workspace

If you edit `src/` inside the container:

```bash
cd /root/arms_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Notes

- The image is intentionally **narrower in scope** than a full development stack: faster builds and less surface area to maintain.
- Repos track branch tips. Rebuild when you want upstream changes; do not expect bit-for-bit identical images across builds.
- If `docker build` fails on `rosdep` or a missing apt package, verify that the repository branches in the `Dockerfile` still match Jazzy.
