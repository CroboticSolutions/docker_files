# RealSense ROS2 Docker Image

Docker image with ROS2 and the [realsense-ros](https://github.com/realsenseai/realsense-ros) wrapper
for Intel/RealSense depth cameras (D400/D500 series, etc.).

`realsense-ros` is ROS2-only (ROS1 support has been discontinued). This image installs
librealsense2 and realsense-ros either from the prebuilt ROS apt packages (default, fast)
or by building both from source (needed for newer camera firmware or the latest driver features).

## Build Arguments

| Argument | Default | Description |
|----------|---------|--------------|
| `ROS_DISTRO` | `jazzy` | ROS2 distro: `humble`, `iron`, `jazzy`, `kilted`, `foxy` |
| `UBUNTU_RELEASE` | `noble` | Ubuntu base image matching `ROS_DISTRO` (see table below) |
| `BUILD_FROM_SOURCE` | `no` | `yes` builds librealsense2 + realsense-ros from source instead of using apt packages |
| `REALSENSE_LIB_VERSION` | `2.58.3-1noble.20260720.173748` | Pinned `ros-jazzy-librealsense2` version (apt path only) — see note below |
| `REALSENSE_CAMERA_VERSION` | `4.58.3-1noble.20260814.095845` | Pinned `ros-jazzy-realsense2-camera` version (apt path only) — see note below |

**Why these are pinned to `ros2-testing` instead of using the stable repo's default:**
the stable `ros2` apt repo's librealsense2/realsense2-camera pairing (`2.58.1`/`4.58.1`)
has a silent `align_depth` bug — the Depth Module opens fine and raw depth publishes,
but `align_depth.enable:=true` (and therefore `aligned_depth_to_color/image_raw`, which
any RGB-D fusion consumer needs) never emits a single frame. No error, no warning, it
just never publishes. Confirmed fixed in the `ros2-testing` repo's `2.58.3`/`4.58.3`
pairing. If bumping these versions, **install the library and wrapper together in one
`apt-get install` call** — doing it as two separate commands (e.g. upgrading just the
library) leaves a version-mismatched pair: the wrapper logs "running with a different
librealsense version than the one it was compiled with" and depth output stops
entirely, not just `align_depth`. Also note `ros2-testing` only keeps the latest build,
not a history — a version pinned here today may disappear from the feed later, same as
`2.58.2` (the version this bug was first confirmed fixed on) had already vanished by the
time `2.58.3` got pinned.

`ROS_DISTRO` and `UBUNTU_RELEASE` must be set together and match:

| ROS_DISTRO | UBUNTU_RELEASE |
|------------|-----------------|
| humble, iron | jammy (22.04) |
| jazzy, kilted | noble (24.04) |
| foxy | focal (20.04) |

## Build

```bash
# Default: Jazzy on Noble, realsense2 packages from apt
docker build -t realsense_img:jazzy .

# Humble on Jammy
docker build --build-arg ROS_DISTRO=humble --build-arg UBUNTU_RELEASE=jammy -t realsense_img:humble .

# Build librealsense2 + realsense-ros from source (e.g. for newer firmware support)
docker build --build-arg BUILD_FROM_SOURCE=yes -t realsense_img:jazzy-src .
```

## Run

Use `run_docker.sh` for the first run (creates the container). Update `IMAGE_NAME` in the
script if you built a different tag than `realsense_img:jazzy`.

```bash
./run_docker.sh
```

Notes:
- `--privileged` and `--volume /dev:/dev` are required so the container can access the
  camera's USB device nodes.
- On the **host**, install librealsense2's udev rules so the camera exposes correct
  permissions before the container even starts (recommended, one-time host setup):
  ```bash
  curl -fsSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/scripts/setup_udev_rules.sh | sudo bash
  ```
- Unplug/replug the camera (or `sudo udevadm trigger`) after installing udev rules.

After the first run, use the usual workflow:
```bash
docker start -i realsense_cont      # resume the container
docker exec -it realsense_cont bash # open another shell into it
docker stop realsense_cont          # stop it
```

## Verifying the camera

Inside the container:

```bash
# Confirm librealsense2 sees the device (installed with realsense2-utils)
rs-enumerate-devices

# Launch the ROS2 camera node (topics, depth/RGB streams, point cloud, etc.)
ros2 launch realsense2_camera rs_launch.py

# With pointcloud 
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

## System Requirements

- Disk space: ~4GB (apt install), ~6GB (build from source)
- A RealSense camera connected via USB