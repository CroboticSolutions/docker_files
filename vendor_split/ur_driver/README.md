# Docker image: ur_driver

Universal Robots ROS 2 driver stack, split out of the combined dev
container's `/root/Dockerfile` — this is that same file minus `arm_api2`.
Brings up the UR driver/description, Gazebo simulation, Robotiq gripper, and
their MoveIt config (`move_group` + controllers). The `arm_api2` container
talks to whatever this container brings up over the shared ROS 2 network —
see [`../README.md`](../README.md).

## What's inside

| Repository | Branch | Remote |
|---|---|---|
| `Universal_Robots_ROS2_Driver` | `multiple_ur_robots` | `CroboticSolutions/Universal_Robots_ROS2_Driver` |
| `Universal_Robots_ROS2_GZ_Simulation` | `multiple_ur_robots` | `CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation` |
| `ros2_robotiq_gripper` | `multiple_ur_robots` | `CroboticSolutions/ros2_robotiq_gripper` |
| `Universal_Robots_ROS2_Description` | `jazzy` | `UniversalRobots/Universal_Robots_ROS2_Description` |
| `serial` (as `serial`) | `master` | `RoverRobotics-forks/serial-ros2` |

These are the branches/remotes currently checked out under
`/root/arms_ws/src` in the combined dev container — update the `Dockerfile`
if that container's branches move on.

**Note on `serial`:** the original combined `/root/Dockerfile` cloned
`tylerjw/serial.git -b ros2`. The dev container's checked-out copy has since
been switched to `RoverRobotics-forks/serial-ros2@master` — this image
follows the fork actually in use today, not the older Dockerfile.

ROS 2 Jazzy `desktop` + MoveIt + `ros2_control` + Gazebo (`gz-ionic`) +
Cyclone DDS. Includes the same Jazzy `robotiq_gripper_controller` namespacing
patch as the original combined Dockerfile.

## Build

```bash
docker build -t ur_driver:latest .
```

## Run

```bash
docker run -it --rm \
  --network host \
  --name ur_driver \
  ur_driver:latest
```

or `bash run_docker.sh`. Not yet built/pushed to Docker Hub, and not yet
wired into `demomotion_gui`'s real launch path (unlike `piper_driver`/
`arm_api2`) -- see [`../README.md`](../README.md#status).

For real hardware, UR's driver talks to the robot controller over TCP/IP on
the same subnet — `--network host` covers that too, no extra `--device`/`/dev`
mounts needed (unlike Piper's CAN adapter).

## Launching

Simulation + MoveIt, single or multiple UR arms:

```bash
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py robots_profile:=lab_gripper_one
```

Real robot (single arm), once you've filled in your robot's IP/kinematics:

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur10e robot_ip:=<robot-ip> launch_rviz:=false
```

Then, from the `arm_api2` container:

```bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=ur robot_ns:=ur1
```

`robot_ns` (here `ur1`) must match whatever namespace this container's
`move_group`/controllers came up under for that arm — see
[`../arm_api2/README.md`](../arm_api2/README.md#gui-robot-selector-alignment).

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
