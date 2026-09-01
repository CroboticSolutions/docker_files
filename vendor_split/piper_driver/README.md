# Docker image: piper_driver

AgileX Piper ROS 2 driver (CAN) + Gazebo simulation + MoveIt bring-up, split
out of the combined dev container. Runs at ROS namespace root by convention
(no per-arm namespace prefix) — matching how the GUI treats Piper as the
workspace-root robot ("Arm"). See [`../README.md`](../README.md).

## What's inside

| Repository | Branch | Remote |
|---|---|---|
| `piper_ros` | `apirsic/devel2` | `CroboticSolutions/piper_ros` |

This is the branch currently checked out under
`/root/arms_ws/src/piper_ros` in the combined dev container — update the
`Dockerfile` if that container's branch moves on. `piper_ros` is a
monorepo: driver node, descriptions (`piper_description` and variants),
`piper_gazebo`, and `piper_moveit` (incl. `demo_arm_api2.launch.py`, built
specifically for `arm_api2` integration) all live inside it.

ROS 2 Jazzy `desktop` + MoveIt + `ros2_control` + Gazebo (`gz-ionic`) +
Cyclone DDS + `can-utils`/`ethtool` + AgileX's `piper_sdk` (pip, not an apt/
rosdep package — the driver nodes `import piper_sdk`).

`piper_ros` also ships an alternative MuJoCo-based simulation package
(`piper_sim/piper_mujoco`), not covered by this image — its
`requirements.txt` pins `mujoco-py==2.1.2.14`, a long-unmaintained package
that's unreliable to `pip install` under Noble's Python and isn't needed for
the Gazebo/MoveIt/real-hardware paths documented below. If you want that
variant, install it separately inside the container rather than baking it
into the `Dockerfile`.

## Build

```bash
docker build -t piper_driver:latest .
```

## Troubleshooting

(31.8.)
**`rosdep update` fails with `HTTP Error 400: Bad Request` fetching
`rosdep/base.yaml`:** this is an upstream GitHub raw-content edge issue, not
this Dockerfile — the same file resolves fine when fetched by a pinned commit
SHA, so the `RUN` step that calls `rosdep init`/`rosdep update` already
resolves the current `rosdistro` `master` SHA via the GitHub API and
re-points `/etc/ros/rosdep/sources.list.d/20-default.list` at that SHA before
updating, working around the broken branch-alias resolution. If it still
fails, GitHub's `api.github.com` may itself be rate-limited/unreachable from
your network — retry, or check https://www.githubstatus.com/.

## Run

**Simulation** — no special flags beyond the usual:

```bash
docker run -it --rm --network host --name piper_driver piper_driver:latest
```

**Real hardware** — the driver talks to the arm over CAN (via a USB-CAN
adapter), which needs privileged access and a visible device node:

```bash
xhost +si:localuser:root   # let the (root) container open windows on your X server
docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name piper_driver \
  piper_driver:latest
```

or `bash run_docker.sh` (defaults to the real-hardware flags above, including
the `xhost`/`DISPLAY`/X11-socket wiring so RViz can actually pop up; drop
`--privileged`/`-v /dev:/dev` for simulation-only use).

RViz is spawned inside this container by the MoveIt launch files
(`demo.launch.py`, `demo_arm_api2.launch.py`) — without the `DISPLAY` env var
and `/tmp/.X11-unix` socket mount above, it has no X server to render to and
either fails silently or errors with something like `qt.qpa.plugin: Could not
load the Qt platform plugin "xcb"`. The `xhost +si:localuser:root` grant is
scoped to local root processes (this container runs as root) rather than the
wider `xhost +local:docker`/`xhost +`.

Pushed to Docker Hub as `croboticsolutions/piper_driver_img:jazzy`.
`demomotion_gui`'s `launch_server` uses this container directly as the real
launch path for the wizard's two real Piper profiles (RealSense D435 and
OAK-D Pro W) -- CAN activation, `piper` driver, MoveIt, and gripper
controller activation all run inside it via `docker exec`, and it pulls this
Hub tag automatically if the local `piper_driver_img:jazzy` image is
missing. See
[`../../../docker_files/composers/vendor_drivers/README.md`](../../../docker_files/composers/vendor_drivers/README.md)
and
[`../../../demomotion_gui/launch_server/hardware_stacks.py`](../../../demomotion_gui/launch_server/hardware_stacks.py).

`--privileged` (or at minimum `--cap-add=NET_ADMIN --cap-add=SYS_ADMIN`
depending on your adapter) is required to bring up the `can0` interface with
`ip link`/`ethtool` from inside the container. Bring the interface up first,
using the scripts shipped in `piper_ros` (`can_activate.sh` etc.), then start
the driver node.

## Launching

Bring up the CAN interface (real hardware only), then the driver + MoveIt
stack:

```bash
# on the host or inside the privileged container, once per boot / adapter replug
bash /root/piper_ws/src/piper_ros/can_activate.sh can0 1000000

# simulation
ros2 launch piper_with_gripper_moveit demo.launch.py

# arm_api2-specific bring-up (matches what the GUI's launch_server invokes)
ros2 launch piper_with_gripper_moveit demo_arm_api2.launch.py

# single real arm
ros2 launch piper start_single_piper.launch.py
```

Then, from the `arm_api2` container:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=piper
```

No `robot_ns` for Piper — it runs at the workspace root, matching
`WORKSPACE_ROOT_ROBOT_ID` in the GUI (see
[`../arm_api2/README.md`](../arm_api2/README.md#gui-robot-selector-alignment)).
If you need multiple Piper arms side by side, you'll need to namespace both
this container's launch and the `arm_api2` launch consistently — the
upstream `piper_ros` also ships `start_two_piper.launch.py` as a starting
point for that.

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
