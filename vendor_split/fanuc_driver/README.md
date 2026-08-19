# Docker image: fanuc_driver

FANUC ROS 2 driver stack — covers both classic FANUC arms (via `fanuc_driver`)
and the CRX cobot family (via `ros2_fanuc_interface`) — plus the welding-cell
bring-up package (`seam_ros2_pkg`) the GUI's `launch_server` actually calls.
Split out of the combined dev container. See [`../README.md`](../README.md).

## What's inside

| Repository | Branch | Remote |
|---|---|---|
| `fanuc_driver` | `apirsic/devel` | `CroboticSolutions/fanuc_driver` |
| `fanuc_description` | `apirsic/devel` | `CroboticSolutions/fanuc_description` |
| `ros2_fanuc_interface` | `paolo` | `paolofrance/ros2_fanuc_interface` |
| `seam_ros2_pkg` | `main` | `CroboticSolutions/seam_ros2_pkg` |

These are the branches currently checked out under `/root/arms_ws/src` in the
combined dev container — update the `Dockerfile` if that container's
branches move on.

`fanuc_driver` is FANUC's own upstream ros2_control streaming driver
(`fanuc_libs`, `fanuc_hardware_interface`, `fanuc_moveit_config`, ...),
targeting Jazzy on its `main` branch. `ros2_fanuc_interface` is a separate,
community-maintained project (not FANUC's) covering the CRX family
(`crx10ia`, `crx10ia_l`, `crx20ia_l`, `crx5ia`, `crx25ia`, `crx30ia`
description + MoveIt configs) — `crx10ia` is one of the `robot_name` values
`arm_api2` already ships config for.

ROS 2 Jazzy `desktop` + MoveIt + `ros2_control` + Gazebo (`gz-ionic` +
`gz_ros2_control`) + Cyclone DDS. `seam_ros2_pkg`'s wrist-camera launch
variants (`crx_real_d435`, `fanuc_hybrid_realsense`, `fanuc_hybrid_oak`,
`fanuc_oak_as_realsense_frames`) pull in `ros-jazzy-realsense2-camera`,
`ros-jazzy-realsense2-description`, and `ros-jazzy-depthai-descriptions-v3`
via `rosdep install` — this is the wrist-mounted camera for the welding cell,
a physically different device from the workspace-view OAK camera that runs
in its own container, so there's no overlap or conflict running both.

## Build

```bash
docker build -t fanuc_driver:latest .
```

## Run

```bash
docker run -it --rm \
  --network host \
  --name fanuc_driver \
  fanuc_driver:latest
```

or `bash run_docker.sh`. Not yet built/pushed to Docker Hub, and not yet
wired into `demomotion_gui`'s real launch path -- FANUC's wizard profiles
still run `ros2 launch` on the host (`ros2` sourced in the shell running
`launch_server`), unchanged. See [`../README.md`](../README.md#status).

Real FANUC hardware talks to the controller over Ethernet (FANUC's streaming
protocol) — `--network host` covers that, no extra device mounts needed.

## Launching

CRX cobot, Gazebo simulation + MoveIt:

```bash
ros2 launch seam_ros2_pkg crx_gz_sim_moveit.launch.py
```

CRX cobot, real robot + RealSense d435:

```bash
ros2 launch seam_ros2_pkg crx_real_d435.launch.py robot_ip:=<robot-ip>
```

Classic FANUC arm, mock/physical control:

```bash
ros2 launch fanuc_hardware_interface fanuc_mock_control.launch.py       # mock
ros2 launch fanuc_hardware_interface fanuc_physical_control.launch.py   # real
ros2 launch seam_ros2_pkg fanuc_moveit_stack.launch.py
```

Then, from the `arm_api2` container:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=crx10ia robot_ns:=<ns>
```

`robot_ns` must match whatever namespace this container's `move_group`/
controllers came up under — see
[`../arm_api2/README.md`](../arm_api2/README.md#gui-robot-selector-alignment).

## Making topics visible to other containers

No bridge is needed. DDS discovers peers over host-network multicast, no port
mapping required. For this container's topics to be visible to `arm_api2` and
the GUI (and vice versa), all of them need:

1. `--network host` / `network_mode: host` (not a custom bridge network)
2. The same `RMW_IMPLEMENTATION` — this image sets `rmw_cyclonedds_cpp`.
3. The same `ROS_DOMAIN_ID` — none of the five containers set one, so they all
   default to domain `0`. If you add it to one, add it to all of them.

The node must also be *launched* in a shell where these vars are already
exported — they're set via `ENV` in the `Dockerfile`, so this holds for every
shell in the container automatically.
