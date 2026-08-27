# RealSense ROS2 Docker Image

Docker image with ROS2 and the [realsense-ros](https://github.com/realsenseai/realsense-ros) wrapper
for Intel/RealSense depth cameras (D400/D500 series, etc.).

`realsense-ros` is ROS2-only (ROS1 support has been discontinued). This image installs
librealsense2 and realsense-ros either by building both from source, pinned to the exact
tags confirmed working with `align_depth` (default), or from the prebuilt ROS apt
packages (opt-in, faster, but **known broken for `align_depth`** — see below).

## Build Arguments

| Argument | Default | Description |
|----------|---------|--------------|
| `ROS_DISTRO` | `jazzy` | ROS2 distro: `humble`, `iron`, `jazzy`, `kilted`, `foxy` |
| `UBUNTU_RELEASE` | `noble` | Ubuntu base image matching `ROS_DISTRO` (see table below) |
| `BUILD_FROM_SOURCE` | `yes` | `no` falls back to the apt packages instead — see the align_depth warning below before using this |
| `REALSENSE_LIB_TAG` | `v2.58.2` | `librealsense` git tag to build (source path only) |
| `REALSENSE_CAMERA_TAG` | `4.58.2` | `realsense-ros` git tag to build (source path only) |

**Why source-built and pinned to these exact tags:** every apt-available pairing has a
silent `align_depth` bug — the Depth Module opens fine and raw depth publishes, but
`align_depth.enable:=true` (and therefore `aligned_depth_to_color/image_raw`, which any
RGB-D fusion consumer needs, e.g. HaMeR hand-mesh tracking) never emits a single frame.
No error, no warning, it just never publishes. This includes both the stable `ros2` apt
repo's `2.58.1`/`4.58.1` pairing *and* the `ros2-testing` repo's `2.58.3`/`4.58.3`
pairing — `2.58.3`/`4.58.3` was briefly believed fixed and pinned here, but reproduced
the exact same silent-drop bug on live testing (see `errors.md`'s "UPDATE 2026-08-25"
and `align_depth_realsense2_camera_fix.md` in `DemoMotion/docs_demo`) — **do not go back
to installing `ros2-testing`'s librealsense2/realsense2-camera for align_depth.** The
only pair confirmed working end-to-end (color + aligned depth + downstream consumers,
verified live) is `librealsense2` `v2.58.2` + `realsense2-camera` `4.58.2` — `2.58.2` is
no longer resolvable via apt at all (`ros2-testing` only keeps its single latest build,
and it's since moved past `2.58.2` with no cached `.deb` anywhere), hence building both
from source pinned to their matching git tags. The source build installs librealsense2
straight into the ROS prefix (`/opt/ros/${ROS_DISTRO}`, not `/usr/local`), so
`realsense2_camera_node` resolves it via the normal `source
/opt/ros/${ROS_DISTRO}/setup.bash` with no extra `LD_LIBRARY_PATH`/`AMENT_PREFIX_PATH`
overlay needed at runtime.

If you do use `BUILD_FROM_SOURCE=no` for a quick non-`align_depth` smoke test, install
the library and wrapper together in one `apt-get install` call — doing it as two
separate commands (e.g. upgrading just the library) leaves a version-mismatched pair:
the wrapper logs "running with a different librealsense version than the one it was
compiled with" and depth output stops entirely, not just `align_depth`.

`ROS_DISTRO` and `UBUNTU_RELEASE` must be set together and match:

| ROS_DISTRO | UBUNTU_RELEASE |
|------------|-----------------|
| humble, iron | jammy (22.04) |
| jazzy, kilted | noble (24.04) |
| foxy | focal (20.04) |

## Build

```bash
# Default: Jazzy on Noble, librealsense2 v2.58.2 + realsense-ros 4.58.2 from source
docker build -t realsense_img:jazzy .

# Humble on Jammy
docker build --build-arg ROS_DISTRO=humble --build-arg UBUNTU_RELEASE=jammy -t realsense_img:humble .

# Apt packages instead (fast, but NOT align_depth-safe -- see warning above)
docker build --build-arg BUILD_FROM_SOURCE=no -t realsense_img:jazzy-apt .
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

- Disk space: ~6GB (build from source, default), ~4GB (`BUILD_FROM_SOURCE=no` apt install)
- A RealSense camera connected via USB