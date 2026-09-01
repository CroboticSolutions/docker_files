# Docker images: arm_api2 + per-vendor drivers

Five separate images, split out of the single combined `/root/Dockerfile` +
`/root/arms_ws` dev container:

| Folder | Image | Contains |
|---|---|---|
| [`arm_api2/`](arm_api2/) | `arm_api2` | `arm_api2` + `arm_api2_msgs` only — the vendor-agnostic MoveIt interface node |
| [`ur_driver/`](ur_driver/) | `ur_driver` | UR driver, description, GZ simulation, Robotiq gripper + their MoveIt/`move_group`/controllers |
| [`piper_driver/`](piper_driver/) | `piper_driver` | AgileX Piper driver (CAN), description, Gazebo, MoveIt |
| [`fanuc_driver/`](fanuc_driver/) | `fanuc_driver` | FANUC streaming driver + CRX cobot family (`ros2_fanuc_interface`) + welding-cell bring-up (`seam_ros2_pkg`) |
| [`abb_driver/`](abb_driver/) | `abb_driver` | **Scaffold only — no working ABB driver exists yet.** Read [`abb_driver/README.md`](abb_driver/README.md) before touching it. |

Each folder is self-contained: its own `Dockerfile`, its own `run_docker.sh`
(except `abb_driver` -- nothing to run yet), and its own `README.md` with
build/run/launch instructions specific to that vendor.

## Status

| Image | Built | Pushed to Docker Hub | Wired into the GUI's real launch path |
|---|---|---|---|
| `arm_api2` | yes | `croboticsolutions/arm_api2_img:jazzy` | yes (Piper real profiles) |
| `piper_driver` | yes | `croboticsolutions/piper_driver_img:jazzy` | yes (Piper real profiles) |
| `ur_driver` | no | no | no |
| `fanuc_driver` | no | no | no |
| `abb_driver` | n/a -- scaffold only | n/a | n/a |

`arm_api2` and `piper_driver` are also driven by
[`docker_files/composers/vendor_drivers/docker-compose.yml`](../../docker_files/composers/vendor_drivers/docker-compose.yml),
which `demomotion_gui`'s `launch_server` (`hardware_stacks.py`) execs into
for the setup wizard's real Piper + RealSense D435 / Piper + OAK-D Pro W
profiles -- see
[`hardware_stacks.py`](../../demomotion_gui/launch_server/hardware_stacks.py)
and [`../../docker_files/composers/vendor_drivers/README.md`](../../docker_files/composers/vendor_drivers/README.md).
`ur_driver`/`fanuc_driver` have compose service definitions ready there too,
but aren't built/pushed, so FANUC's wizard profiles still run on the host
(`ros2` sourced in the shell running `launch_server`), unchanged.

## Why this split

In the combined container, the tutorial flow is already two independent
pieces talking over ROS 2, not one monolithic program:

- **Terminal 1** brings up simulation/hardware + MoveIt (`move_group` +
  `ros2_control` controllers) for one vendor.
- **Terminal 2** runs `arm_api2`'s interface node, which talks to whatever
  `move_group` is already running via `MoveGroupInterface` — it doesn't care
  which vendor is behind it.

That's exactly the boundary these five containers now enforce structurally:
`arm_api2` never bundles a vendor driver, and each driver container never
bundles `arm_api2`.

## Branches

Every `Dockerfile` clones the exact branch currently checked out under
`/root/arms_ws/src` in this dev container (not necessarily the same branch
the original combined `/root/Dockerfile` used — e.g. `arm_api2` has moved
from `jazzy` to `apirsic/devel`, and UR's `serial` dependency has moved from
`tylerjw/serial@ros2` to `RoverRobotics-forks/serial-ros2@master`). Each
per-folder README has a table of exactly what it clones. If the dev
container's branches move on, update the corresponding `Dockerfile`.

## Networking contract (all five containers, plus the GUI)

No bridge is needed between any of these containers, or between them and the
GUI — this mirrors how the OAK camera container's topics are already made
visible to `demomotion`'s containers. DDS discovers peers over host-network
multicast, no port mapping required. For any two of these containers (or the
GUI) to see each other's topics/services, **all** of them need:

1. **`--network host` / `network_mode: host`** (not a custom bridge network) —
   every `Dockerfile` here assumes this; none of them `EXPOSE` a port.
2. **The same `RMW_IMPLEMENTATION`** — all five set `rmw_cyclonedds_cpp` via
   `ENV`. A FastDDS container and a CycloneDDS container will not discover
   each other even on the same host network.
3. **The same `ROS_DOMAIN_ID`** — none of the five set one, so they all
   default to domain `0`. If you ever add `ROS_DOMAIN_ID` to one of them, add
   the identical value to *all* of them (including the GUI/rosbridge
   container), or they'll silently stop seeing each other.

The node in each container must also be *launched* in a shell where these
vars are already exported — a running process inherits env vars at launch
time, so sourcing `.bashrc` after the fact has no effect on it. Since all
three are set via `ENV` in each `Dockerfile`, this already holds for every
shell in every one of these containers.

rosbridge (the websocket on port `9094` the GUI's frontend connects to,
`ws://<host>:9094/ros`) is **not** one of these five containers — it's
expected to run in the existing GUI/backend container. It needs to join the
same host network / RMW / domain as everything above for the GUI to see any
of these five.

## Making a robot show up in the GUI's robot selector

`arm_api2_gui`'s `RobotSelector` (`RobotSelector.tsx`,
`RobotSelectorContext.tsx`, under `src/components/robot/` in the
[`arm_api2_gui`](https://github.com/CroboticSolutions/arm_api2_gui) repo --
paths not re-checked against this checkout; the original write-up assumed a
devcontainer layout that isn't present here)
discovers robots by probing ROS namespaces of the form
`${robot_namespace_prefix}${i}` (`i` = 1..`max_robots`) over the rosbridge
websocket. The prefix comes from the GUI's World Setup wizard
(falls back to `config/ros2_config.json`'s `robot_namespace_prefix`, which is
currently `""` — meaning single-robot-at-root, shown as **Arm**, matching
Piper's convention).

For a given robot instance to appear correctly:

- The `robot_ns` passed to `arm_api2 moveit2_iface.launch.py` (in the
  `arm_api2` container) and the namespace the matching driver container
  brings its `move_group`/controllers up under **must be the identical
  string**. `arm_api2` publishes/subscribes the GUI-facing topics
  (`current_pose`, `joint_states`, `arm/cmd/pose`, ...) under that namespace;
  if the two containers disagree on it, the GUI will either not find the
  robot or find an empty/stale one.
- Multiple simultaneous robots (e.g. `ur1` + `ur2`, or `ur1` + `piper1`) need
  a shared prefix set in the wizard, with `<prefix>1`, `<prefix>2`, ...
  applied consistently to both the `arm_api2` launch and each driver
  container's launch.
- The single-robot case (no `robot_ns`, workspace root) is what Piper already
  defaults to; UR/Fanuc/ABB can run the same way if you're only ever bringing
  up one arm at a time.

This applies regardless of which of the five containers each half is running
in — the selector only ever looks at ROS namespaces on the DDS network, not
at which container produced them.

## ABB

Not installed anywhere in this workspace, and no maintained ROS 2 driver was
found to slot in. `abb_driver/` is a labelled scaffold, not a working image —
see [`abb_driver/README.md`](abb_driver/README.md) for what's actually there,
what was checked, and what to decide before filling in its `Dockerfile`.
