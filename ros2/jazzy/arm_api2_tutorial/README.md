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

## Build and run

From the directory that contains `Dockerfile` and `first-run.sh`:

1. **Build** the image:

   ```bash
   docker build -t arm_api2:tutorial .
   ```

2. **Run** the container (via `first-run.sh` in this repo):

   ```bash
   chmod +x first-run.sh   # once
   ./first-run.sh
   ```

   What `first-run.sh` does (edit the script if you want other names or flags):

   - **Image:** `arm_api2:tutorial`
   - **Container name:** `arm_api2_tutorial`
   - **`docker run`:** `-it`, `--network host`, `--privileged`, bind-mount `/dev`, bind-mount `/tmp/.x11-unix` → `/tmp/.x11-unix`, `DISPLAY`, `TERM=xterm-256color`, interactive `/bin/bash`
   - **SSH agent:** `ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock` on the host, then mount `~/.ssh/ssh_auth_sock` → `/ssh-agent` with `SSH_AUTH_SOCK=/ssh-agent` ([tmux / VS Code forwarding notes](https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/))

   Before the first run, create `~/.ssh` on the host if it does not exist. If a container with the same name is already there, remove it: `docker rm -f arm_api2_tutorial`.

   For **Gazebo / RViz** on the host you may need: `xhost +local:root`.  
   Many Linux systems expose the X11 socket as **`/tmp/.X11-unix`** (capital `X`); if the GUI does not show, align the host path in `first-run.sh` with your system.

   A **second shell** in the same container:

   ```bash
   docker exec -it arm_api2_tutorial /bin/bash
   ```

Another distro when building (rare):

```bash
docker build -t arm_api2:tutorial --build-arg ROS2_DISTRO=jazzy .
```

## Tutorial: two terminals in the container

Inside the container you get interactive `/bin/bash`; the image’s `/root/.bashrc` sources ROS Jazzy and `/root/arms_ws/install/setup.bash`, so that environment should load automatically. Then follow the [official tutorial README](https://github.com/CroboticSolutions/arm_api2/blob/jazzy/tutorials/README.md).

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
