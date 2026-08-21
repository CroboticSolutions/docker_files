# Vendor Driver + arm_api2 Compose

One service per container in the split-out `arm_api2` + per-vendor driver
set (see
[`../../vendor_split/README.md`](../../vendor_split/README.md)
for the full design and networking contract). Unlike the old
`composers/arm_api2` / `composers/cameras` smoke tests, this compose file is
the **actual launch path** for the setup wizard's real Piper profiles --
`demomotion_gui/launch_server`'s `hardware_stacks.py` execs into these
containers via `docker exec` instead of assuming `ros2` is sourced on the
host. See
[`../../../demomotion_gui/launch_server/hardware_stacks.py`](../../../demomotion_gui/launch_server/hardware_stacks.py).

Services:
- **arm_api2** -- [`../../vendor_split/arm_api2`](../../vendor_split/arm_api2) (vendor-agnostic MoveIt interface, Jazzy)
- **piper_driver** -- [`../../vendor_split/piper_driver`](../../vendor_split/piper_driver) (AgileX Piper CAN driver + Gazebo + MoveIt)
- **realsense** -- [`../../realsense`](../../realsense) (Intel RealSense D400 series; used by the PiPER + RealSense D435 wizard profile so the camera launch runs in-container like `piper_driver`/`arm_api2`, instead of needing ROS sourced on the host)
- **perception** -- `croboticsolutions/perception:jazzy` on Docker Hub, no local Dockerfile (see Build below) -- HaMeR/WiLoR hand-pose + SAM for the WELDING application's `mp_wrapper_ros` nodes and the `/teleop/start-hand-servo` endpoint. `docker exec`'d into from
  [`../../../demomotion_gui/launch_server/welding_stack.py`](../../../demomotion_gui/launch_server/welding_stack.py)
  and `main.py`, not `hardware_stacks.py`'s wizard profiles -- see
  `docs_demo/perception_container_setup.md` in `DemoMotion` for the full
  writeup (a bug in the published image needing a live per-container patch
  until it's fixed upstream: missing `libgles2`, missing `arm_api2_msgs`,
  `mp_wrapper_ros`'s unconditional `pynput` import needing a virtual
  display).
- **ur_driver** -- [`../../vendor_split/ur_driver`](../../vendor_split/ur_driver) (Universal Robots driver + Gazebo + MoveIt) -- defined here, **not yet built/pushed**
- **fanuc_driver** -- [`../../vendor_split/fanuc_driver`](../../vendor_split/fanuc_driver) (FANUC streaming driver + CRX cobot family + welding cell) -- defined here, **not yet built/pushed**

`abb_driver` has no service here -- its `Dockerfile` is a deliberate `exit 1`
scaffold (no working ABB ROS 2 driver exists yet); see
[`../../vendor_split/abb_driver/README.md`](../../vendor_split/abb_driver/README.md).

## Build

```bash
docker compose -f docker-compose.yml build arm_api2 piper_driver realsense
# ur_driver / fanuc_driver build fine too, but are large (full ROS 2 desktop +
# Gazebo + MoveIt) -- check free disk before building them.
```

`perception` has no `build:` section -- it's pulled straight from Docker Hub
and re-tagged locally, same pull+tag fallback the others use if their local
image is missing (see `_pull_and_tag_docker_image` in `main.py`), just
without a local source to fall back to *building* from if the pull ever
fails. It also needs `runtime: nvidia` (the host's Docker daemon needs the
nvidia container runtime configured, e.g. via `nvidia-container-toolkit`) --
its CUDA-based hand-pose models are impractically slow on CPU.

## Run

`hardware_stacks.py` starts these automatically (`docker compose up -d
<service>`) the first time a profile that needs them is launched from the
wizard, pulling the matching image from Docker Hub first (see table below) if
the local image is missing, and only falling back to a local `docker compose
build` if that pull fails. To start one manually:

```bash
docker compose -f docker-compose.yml up -d arm_api2 piper_driver realsense
```

Containers are meant to stay up (idle `bash`) between wizard launches --
`hardware_stacks.py` only starts/stops the `ros2 launch` process trees
inside them (`docker exec <container> pkill -f <pattern>`), not the
containers themselves.

## Docker Hub

| Image | Tag |
|---|---|
| `arm_api2` | `croboticsolutions/arm_api2:jazzy` |
| `piper_driver` | `croboticsolutions/piper_driver:jazzy` |
| `realsense` | `croboticsolutions/realsense_img:jazzy` |
| `perception` | `croboticsolutions/perception:jazzy` |

Note: `croboticsolutions/arm_api2:humble` is a **different, older** image
(the combined dev-container-style build) -- this split, Jazzy-only
`arm_api2` is a separate tag, not a replacement for it.

`croboticsolutions/oak_img:humble` is also published, but is **not** wired
into `hardware_stacks.py`'s `CONTAINER_HUB_IMAGE`/`CONTAINER_COMPOSE_SERVICE`
yet -- the OAK-D Pro W wizard profile's camera step launches
`depthai_ros_driver_v3`'s `rgbd_pro_w_pcl.launch.py`, a different
package/launch file than the OAK-D SR-only `depthai_ros_driver` workspace
`../../luxonis` builds, so it needs its own verified compose service before
it can be containerized the same way.
