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