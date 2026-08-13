# Luxonis OAK-D SR (depthai-ros, ROS2 Humble)

Builds [depthai-ros](https://github.com/luxonis/depthai-ros) (`humble` branch, v2.12.2)
for an **OAK-D SR** camera (stereo-only, no RGB sensor).

## 1. Host setup (one-time, run on the HOST, not in the container)

```bash
./install_udev_rules_host.sh
```

Grants non-root USB access to the camera. The container runs `--privileged` with
`/dev:/dev` mounted, but the udev rule itself only needs to exist on the host --
containers have no udev daemon of their own.

## 2. Build

```bash
docker build -t ros2_oak_sr:humble .
```

## 3. Run

Either standalone:

```bash
./run_docker.sh
```

or as the `oak_sr` service in [`../composers/demomotion/docker-compose.yml`](../composers/demomotion/docker-compose.yml)
-- see that folder's README for the compose workflow.

## 4. Launch the driver

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /root/depthai_ws/install/setup.bash
ros2 launch depthai_ros_driver sr_rgbd_pcl.launch.py rectify_rgb:=true
```

`sr_rgbd_pcl.launch.py` (not `rgbd_pcl.launch.py`) is the SR-specific launch file --
`rgbd_pcl.launch.py` hardcodes the point cloud node to `/oak/rgb/image_rect`, which
doesn't exist on SR (no RGB camera). `rectify_rgb:=true` loads an
`image_proc::RectifyNode` that undistorts `/oak/right/image_raw` -> `/oak/right/image_rect`.

The driver auto-detects the OAK-D SR and aligns stereo depth to **CAM_C (right)**, so
`/oak/right/image_rect` and `/oak/stereo/image_raw` share the same optical frame.

## What was changed vs. the upstream default

**`to_copy/oak_d_sr.yaml`** (copied over `depthai_ros_driver/config/oak_d_sr.yaml` at
build time):

```yaml
/oak:
  ros__parameters:
    camera:
      i_pipeline_type: 'Depth'   # SR has no RGB -> forces Depth pipeline
    right:
      i_publish_topic: true       # publish raw right mono image
    stereo:
      i_right_rect_publish_topic: true  # publish rectified right image
```

The `stereo.i_right_rect_publish_topic` line is the only addition over upstream's
default SR config -- it makes the rectified right stereo image available directly
from the driver, without a separate `image_proc` node.

**`depthai_ros_driver/package.xml`** and **`depthai-ros/package.xml`**: the
`depthai_examples` dependency is stripped (via `sed`, during the Docker build) --
that package isn't needed to run the driver and breaks the build in this image.

## Topics published

| Topic | Content |
|---|---|
| `/oak/right/image_raw` | Raw right mono (distorted) |
| `/oak/right/image_rect` | Rectified right mono |
| `/oak/right/camera_info` | Right camera intrinsics |
| `/oak/stereo/image_raw` | Depth (uint16, mm), aligned to right frame |
| `/oak/stereo/camera_info` | Depth camera info (frame: `oak_right_camera_optical_frame`) |
| `/oak/points` | Colored point cloud (grey) |
| `/oak/imu/data` | IMU |

All topics are published as `BEST_EFFORT` QoS -- set **Reliability Policy -> Best
Effort** in RViz for any Image/PointCloud2 display, and use `BEST_EFFORT` for any
subscriber you add.

**Verified on real hardware** (2026-08-13, OAK-D-SR, MXID `19443010B17E782700`):
driver detects the camera as `OAK-D-SR` / `Pipeline type: Depth`, `/oak/right/image_rect`
runs ~20-26 Hz and `/oak/stereo/image_raw` a steady ~30 Hz, with real intrinsics on
`/oak/right/camera_info`. The launch file also advertises `/oak/rgb/*` topics (generic
pipeline scaffolding) -- these never publish any data on SR, since it has no RGB sensor;
ignore them.

## Making the OAK topics visible from other containers (e.g. demomotion)

No bridge is needed. `arm_api2_ur` and `mediapipe` in
[`../composers/demomotion/docker-compose.yml`](../composers/demomotion/docker-compose.yml)
already run with `network_mode: host`, which is how every container in this repo
talks to ROS2 -- DDS discovers peers over host-network multicast, no port mapping
required. For a new container's topics to show up there too, it only needs:

1. `network_mode: host` (not a custom bridge network)
2. The same `RMW_IMPLEMENTATION` -- this image sets `rmw_cyclonedds_cpp`, matching
   `arm_api2_ur`/`mediapipe`. A FastDDS container and a CycloneDDS container will
   not discover each other even on the same host network.
3. The same `ROS_DOMAIN_ID` -- none of the three services set one, so all three
   default to domain `0`. If you ever add `ROS_DOMAIN_ID` to one, add the same
   value to all of them, or they'll stop seeing each other.

The node must also be *launched* in a shell where these vars are already exported --
a running process inherits env vars at launch time, so sourcing `.bashrc` afterwards
has no effect on it. Since both are set via `ENV` in the Dockerfile, this is already
the case for every shell in the container.

**Caveat:** `/oak/points` is several MB/frame and fragments badly over DDS across
containers (~2 Hz instead of ~10 Hz). If another container needs the point cloud,
prefer subscribing to it from a node in *this* container instead of across the
network.
