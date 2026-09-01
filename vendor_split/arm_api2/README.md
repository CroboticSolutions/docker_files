# Docker image: arm_api2

Vendor-agnostic `arm_api2` interface node, split out of the combined dev
container's `/root/Dockerfile`. This image is deliberately thin: it only
contains `arm_api2` + `arm_api2_msgs`. It has no vendor driver, no
`ros2_control`, no Gazebo — those live in `ur_driver`, `piper_driver`,
`fanuc_driver`, `abb_driver` (see [`../README.md`](../README.md) for how the
five containers fit together).

## What's inside

| Repository | Branch | Remote |
|---|---|---|
| `arm_api2` | `apirsic/devel` | `CroboticSolutions/arm_api2` |
| `arm_api2_msgs` | `apirsic/devel` | `CroboticSolutions/arm_api2_msgs` |

These are the branches currently checked out under `/root/arms_ws/src` in the
combined dev container — update the `Dockerfile` if that container's branches
move on.

ROS 2 Jazzy `ros-base` + MoveIt (`MoveGroupInterface` client libraries) +
Cyclone DDS.

## Build

```bash
docker build -t arm_api2:latest .
```

## Run

```bash
docker run -it --rm \
  --network host \
  --name arm_api2 \
  arm_api2:latest
```

or `bash run_docker.sh` (same command, kept in sync with this README).

Pushed to Docker Hub as `croboticsolutions/arm_api2_img:jazzy` -- distinct from
the older, unrelated `croboticsolutions/arm_api2:humble` combined-container
build. `demomotion_gui`'s `launch_server` pulls this tag automatically
(via `docker_files/composers/vendor_drivers`) if the local
`arm_api2_img:jazzy` image is missing when the wizard starts a real Piper
profile -- see
[`../../../docker_files/composers/vendor_drivers/README.md`](../../../docker_files/composers/vendor_drivers/README.md).

`--network host` is required, not optional — see
[Making topics visible to other containers](#making-topics-visible-to-other-containers)
below.

## Launching the interface

Inside the container, once the corresponding driver container (e.g.
`ur_driver`) already has `move_group` and controllers up for a given
namespace:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur robot_ns:=ur1
```

`robot_name` selects which config under `arm_api2`'s `config/` folder to load
(`ur`, `piper`, `fanuc`/`crx10ia`, `abb`, `kinova`, `franka`, `so_arm100`, ...).
`robot_ns` **must match** the ROS namespace the driver container brought its
`move_group`/controllers up under — see
[GUI robot selector alignment](#gui-robot-selector-alignment).

For Piper (workspace-root convention, no namespace), omit `robot_ns` or pass
an empty one, matching how `piper_driver` brings itself up.

## Making topics visible to other containers (e.g. driver containers, the GUI)

No bridge is needed. DDS discovers peers over host-network multicast, no port
mapping required. For this container's topics to be visible to
`ur_driver`/`piper_driver`/`fanuc_driver`/`abb_driver` and the GUI, and vice
versa, all of them need:

1. `--network host` / `network_mode: host` (not a custom bridge network)
2. The same `RMW_IMPLEMENTATION` — this image sets `rmw_cyclonedds_cpp`. A
   FastDDS container and a CycloneDDS container will not discover each other
   even on the same host network.
3. The same `ROS_DOMAIN_ID` — none of the five containers set one, so they all
   default to domain `0`. If you ever add `ROS_DOMAIN_ID` to one, add the same
   value to all of them, or they'll stop seeing each other.

The node must also be *launched* in a shell where these vars are already
exported — a running process inherits env vars at launch time, so sourcing
`.bashrc` afterwards has no effect on it. Since both are set via `ENV` in the
`Dockerfile`, this is already the case for every shell in the container.

## GUI robot selector alignment

`arm_api2_gui`'s `RobotSelector` (see
`arms_ws/src/arm_api2_gui/src/components/robot/RobotSelector.tsx` and
`RobotSelectorContext.tsx`) discovers robots by probing ROS namespaces of the
form `${robot_namespace_prefix}${i}` (`i` = 1..`max_robots`) over the
rosbridge websocket, using the prefix configured in the GUI's World Setup
wizard (`config/ros2_config.json`'s `robot_namespace_prefix`, currently `""`
→ single robot at the workspace root, shown as **Arm**).

For a robot to show up in that selector:

- The `robot_ns` you pass to `arm_api2`'s `moveit2_iface.launch.py` and the
  namespace the corresponding driver container brings its stack up under
  **must be identical**.
- If you want multiple simultaneous robots (e.g. `ur1` + `ur2`, or `ur1` +
  `piper1`), set a shared `robot_namespace_prefix` in the wizard and use
  `<prefix>1`, `<prefix>2`, ... consistently across both the `arm_api2`
  launch and the driver container's launch for each instance.
- Leaving everything at the root namespace (no `robot_ns`) is the
  single-robot case and matches Piper's default convention.

rosbridge itself (the websocket on port `9094` the GUI connects to) is not
part of this image — it's expected to already be running (in the GUI/backend
container) on the same host network with the same `RMW_IMPLEMENTATION`/
`ROS_DOMAIN_ID`.
