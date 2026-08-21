# Docker image: abb_driver — NOT READY, read this first

Short version: **there is no ABB ROS 2 driver installed anywhere in this
setup, and no obvious one to install.** The `Dockerfile` in this folder is a
scaffold (same base layer as `ur_driver`/`piper_driver`/`fanuc_driver`) with a
deliberate `exit 1` where the driver clone would go, so it can't be built by
accident into something that looks done but isn't.

## What's actually here today

| Path | What it is | A live driver? |
|---|---|---|
| `/root/abb_irb6700_145_320_gripper_moveit_config` | MoveIt config + URDF for an IRB6700, single and dual-arm (`config/multi_abb/`) | No — sim/planning config only |
| `arm_api2`'s `config/abb/*.yaml` | Kinematics config so `arm_api2` can run with `robot_name:=abb` | No — talks to whatever `move_group`/controllers exist under that namespace, there just aren't any for ABB yet |

Neither brings up a `ros2_control` hardware interface or talks to a real ABB
controller. `arm_api2` listing `abb` as a supported `robot_name` describes
config-file support, not a working hardware/sim bring-up.

## What I checked and why it's not in the Dockerfile

The obvious candidate, **`ros-industrial/abb_robot_driver`**, is **ROS 1**
(`catkin`/`catkin_tools`, tested on Melodic; CI covers Bionic/Focal). It
cannot be built into this ROS 2 Jazzy / Ubuntu Noble stack — trying to
`rosdep install` or `colcon build` it will fail outright, not just warn.
A GitHub search for actively maintained ROS 2 ABB drivers (EGM/RWS-based or
otherwise) didn't turn up a maintained equivalent — what exists is scattered,
low-activity, per-robot student/hobby projects (single-digit stars, one-off
IRB models), not something to silently wire into a Dockerfile you'll rely on.

## Before you fill in the TODO

1. **Pick (or confirm) the actual driver.** If Crobotics already has a
   private fork or a different plan for ABB (e.g. wrapping ABB's own
   `EGM`/`RWS` C++ SDKs directly, or a `ros2_control` hardware interface
   written in-house, similar in shape to `fanuc_driver`), point the
   `Dockerfile`'s `WORKDIR /root/abb_ws/src` step at it instead of the
   placeholder `exit 1`.
2. **Check ROS 2 distro support explicitly** — don't assume Jazzy just
   because the rest of this stack is Jazzy; ABB driver projects skew ROS 1
   or Humble.
3. **Carry over the existing config** either way:
   - Copy `abb_irb6700_145_320_gripper_moveit_config` in as a source package
     if you want the IRB6700-specific MoveIt config, or use whatever
     MoveIt config ships with the driver you pick.
   - `arm_api2`'s `config/abb/*.yaml` already lives in the `arm_api2`
     container and needs no change — it just needs a real `move_group` +
     controllers to talk to under the `abb` namespace, which is this
     container's job once it exists.
4. **Follow the same networking contract as the other four** once it builds
   — `--network host`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and the same
   `ROS_DOMAIN_ID` as everything else. See
   [`../README.md`](../README.md) and
   [`../arm_api2/README.md`](../arm_api2/README.md#making-topics-visible-to-other-containers).

## Once it's real

Update this README's "What's inside" (branches/remotes, like the other four
folders) and drop this banner.
