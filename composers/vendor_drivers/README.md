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
- **perception** -- this service still runs `croboticsolutions/perception:jazzy`
  from Docker Hub (see Build below) -- HaMeR/WiLoR hand-pose + SAM for the
  WELDING application's `mp_wrapper_ros` nodes and the
  `/teleop/start-hand-servo` endpoint. `docker exec`'d into from
  [`../../../demomotion_gui/launch_server/welding_stack.py`](../../../demomotion_gui/launch_server/welding_stack.py)
  and `main.py`, not `hardware_stacks.py`'s wizard profiles -- see
  `docs_demo/perception_container_setup.md` in `DemoMotion` for the full
  writeup of three runtime bugs in the published image (missing `libgles2`,
  missing `arm_api2_msgs`, `mp_wrapper_ros`'s unconditional `pynput` import
  needing a virtual display) that previously needed a live per-container
  patch to work around. **[`../../perception`](../../perception) now has a
  from-scratch local Dockerfile with all three baked in** (plus a pinned CUDA
  base image, checksummed model downloads, and a non-root runtime user) --
  not yet wired as this service's `build:` source, so `perception_img:jazzy`
  here is still the unpatched Hub image until that's decided.
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

`perception` has no `build:` section here -- it's pulled straight from
Docker Hub and re-tagged locally, same pull+tag fallback the others use if
their local image is missing (see `_pull_and_tag_docker_image` in
`main.py`), just without a local source to fall back to *building* from if
the pull ever fails -- [`../../perception`](../../perception) exists as a
standalone Dockerfile/`run_docker.sh` (see its own README) but isn't
referenced from this compose file yet. It also needs `runtime: nvidia` (the
host's Docker daemon needs the nvidia container runtime configured, e.g. via
`nvidia-container-toolkit`) -- its CUDA-based hand-pose models are
impractically slow on CPU.

## Run

`hardware_stacks.py` starts these automatically (`docker compose up -d
<service>`) the first time a profile that needs them is launched from the
wizard, pulling the matching image from Docker Hub first (see table below) if
the local image is missing, and only falling back to a local `docker compose
build` if that pull fails. To start one manually:

```bash
./docker-compose-up.sh -f docker-compose.yml up -d arm_api2 piper_driver realsense
```

`docker-compose-up.sh` is a thin wrapper (same pattern as
`../demomotion/docker-compose-up.sh`) that runs `xhost +si:localuser:root`
before `docker compose "$@"`, so RViz (launched inside `piper_driver_cont` by
the MoveIt launch files) can actually open a window -- `DISPLAY` and
`/tmp/.X11-unix` are already mounted into these containers below, but nothing
authorizes them against the host's X server otherwise. **Note:**
`hardware_stacks.py` currently calls `docker compose` directly, not this
wrapper, so the wizard's auto-start path still needs either that script
pointed at this wrapper or an equivalent `xhost` grant done once per login on
the host (e.g. in `~/.xprofile`) to get the same effect.

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
