# DemoMotion Docker Compose

Host-networked stack. The five Hub images are **pulled**, not built.
MediaPipe lives inside **perception** (`mp_wrapper_ros`).

| Service | Image | Distro | Container | Role |
|---|---|---|---|---|
| `arm_api2_humble` | `croboticsolutions/arm_api2:humble-selftest` | Humble | `arm_api2_cont_humble` | UR + MoveIt + `arm_api2` |
| `perception` | `croboticsolutions/perception:jazzy` | Jazzy GPU | `arm-api2-perception` | MediaPipe / WiLoR, SAM, hand-eye |
| `realsense` | `croboticsolutions/realsense_img:jazzy` | Jazzy | `realsense_cont` | Intel RealSense D435 / D400 |
| `oak` | `croboticsolutions/oak_img:humble` | Humble | `oak_cont` | Luxonis OAK-D (depthai-ros) |
| `piper_driver` | `croboticsolutions/piper_driver:jazzy` | Jazzy | `piper_driver_cont` | AgileX Piper driver + MoveIt |

Local-only (not on Docker Hub — skip these unless you need them):

| Service | Image | Distro |
|---|---|---|
| `ur_driver` | build [`../../vendor_split/ur_driver`](../../vendor_split/ur_driver) | Jazzy |
| `fanuc_driver` | build [`../../vendor_split/fanuc_driver`](../../vendor_split/fanuc_driver) | Jazzy |

`abb_driver` is omitted (`exit 1` scaffold). There is no Jazzy
`vendor_split/arm_api2` — the interface is `arm_api2_humble` only.

Do **not** run `./docker-compose-up.sh pull` or `up -d` with no service names.
Compose then tries `ur_driver_img` / `fanuc_driver_img` on Docker Hub and
fails with `pull access denied`.

## Quick start (all Hub containers)

From this directory:

```bash
./docker-compose-up.sh pull arm_api2_humble perception realsense oak piper_driver
./docker-compose-up.sh up -d arm_api2_humble perception realsense oak piper_driver
```

That pulls (if missing) and starts all five: Humble `arm_api2`, perception,
RealSense, OAK, Piper. Containers stay up with idle `bash`. Launch ROS
nodes yourself via `docker exec` (examples below).

`pull_policy: missing` — `up` will not re-download a tag that is already
local. Re-run the `pull` line to refresh from Hub.

Stop:

```bash
./docker-compose-up.sh down
```

## Prerequisites

- Docker Compose v2
- NVIDIA Container Toolkit (`perception` uses `--gpus all`)
- Host udev, once, if you plug in a camera:
  - OAK: `../../luxonis/install_udev_rules_host.sh`
  - RealSense: `curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/scripts/setup_udev_rules.sh | sudo bash`

Hub repos under `croboticsolutions/` are public. Pull does not need GitHub SSH.

## Networking

Every service uses `network_mode: host`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
and `ROS_DOMAIN_ID` default `0`. Override for the whole stack:

```bash
ROS_DOMAIN_ID=42 ./docker-compose-up.sh up -d arm_api2_humble perception realsense oak piper_driver
```

If you set the domain on one service, set the same value on every peer
(including the GUI/rosbridge container), or DDS discovery fails silently.

## Attach

```bash
docker exec -it arm_api2_cont_humble bash
docker exec -it arm-api2-perception bash
docker exec -it realsense_cont bash
docker exec -it oak_cont bash
docker exec -it piper_driver_cont bash
```

## Launch examples

Containers do not start ROS nodes by themselves.

### Cameras

OAK-D SR (inside `oak_cont`):

```bash
source /opt/ros/humble/setup.bash
source /root/depthai_ws/install/setup.bash
ros2 launch depthai_ros_driver sr_rgbd_pcl.launch.py rectify_rgb:=true
```

RealSense (inside `realsense_cont`):

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

OAK and RealSense can both be running as idle containers; start the driver
only in the camera you are using.

### Perception (inside `arm-api2-perception`)

Camera topics must already be publishing.

```bash
ros2 launch mp_wrapper_ros hamer_mp_hamer_oak_d_pro_w.launch.py
ros2 launch sam_ros2 sam_interactive.launch.py robot:=piper
ros2 launch hand_eye_calibration calibration.launch.py
```

### Humble arm (`arm_api2_cont_humble`)

```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
ros2 launch arm_api2 moveit2_iface.launch.py \
  robot_name:=ur use_sim_time:=true mode:=advanced
ros2 run arm_api2 arm_api2_selftest.py
```

The Hub `humble-selftest` image may be built without Gazebo. If
`ur_simulation_gz` is missing, use a real UR or an already-running sim.

### Piper (`piper_driver_cont`, then Humble `arm_api2`)

Piper uses workspace root (no `robot_ns`). Bring the driver up first, then:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=piper
```

Details: [`../../vendor_split/README.md`](../../vendor_split/README.md).

Verify DDS from any container:

```bash
ros2 topic list
```

## UR / FANUC (local build only)

Not on Docker Hub. Only if you need them:

```bash
./docker-compose-up.sh build ur_driver
./docker-compose-up.sh up -d ur_driver

./docker-compose-up.sh build fanuc_driver
./docker-compose-up.sh up -d fanuc_driver
```

Then, matching namespaces:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur robot_ns:=ur1
```
