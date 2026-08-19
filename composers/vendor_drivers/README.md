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
- **ur_driver** -- [`../../vendor_split/ur_driver`](../../vendor_split/ur_driver) (Universal Robots driver + Gazebo + MoveIt) -- defined here, **not yet built/pushed**
- **fanuc_driver** -- [`../../vendor_split/fanuc_driver`](../../vendor_split/fanuc_driver) (FANUC streaming driver + CRX cobot family + welding cell) -- defined here, **not yet built/pushed**

`abb_driver` has no service here -- its `Dockerfile` is a deliberate `exit 1`
scaffold (no working ABB ROS 2 driver exists yet); see
[`../../vendor_split/abb_driver/README.md`](../../vendor_split/abb_driver/README.md).

## Build

```bash
docker compose -f docker-compose.yml build arm_api2 piper_driver
# ur_driver / fanuc_driver build fine too, but are large (full ROS 2 desktop +
# Gazebo + MoveIt) -- check free disk before building them.
```

## Run

`hardware_stacks.py` starts these automatically (`docker compose up -d
<service>`) the first time a profile that needs them is launched from the
wizard, pulling `croboticsolutions/<name>:jazzy` from Docker Hub first if the
local image is missing. To start one manually:

```bash
docker compose -f docker-compose.yml up -d arm_api2 piper_driver
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

Note: `croboticsolutions/arm_api2:humble` is a **different, older** image
(the combined dev-container-style build) -- this split, Jazzy-only
`arm_api2` is a separate tag, not a replacement for it.
