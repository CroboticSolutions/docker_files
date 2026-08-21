# Vendor driver compose

Per-vendor driver containers from [`../../vendor_split`](../../vendor_split).
The MoveIt interface is **not** here — `vendor_split/arm_api2` was removed.
Use [`../demomotion`](../demomotion) `arm_api2_humble` (`ros2/humble/arm_api2_with_test`).

Networking: host network + `rmw_cyclonedds_cpp` + the same `ROS_DOMAIN_ID` as
`arm_api2_humble` and the GUI/rosbridge container. See
[`../../vendor_split/README.md`](../../vendor_split/README.md).

Services:
- **piper_driver** -- [`../../vendor_split/piper_driver`](../../vendor_split/piper_driver) (AgileX Piper CAN driver + Gazebo + MoveIt)
- **ur_driver** -- [`../../vendor_split/ur_driver`](../../vendor_split/ur_driver) (Universal Robots driver + Gazebo + MoveIt)
- **fanuc_driver** -- [`../../vendor_split/fanuc_driver`](../../vendor_split/fanuc_driver) (FANUC streaming driver + CRX cobot family + welding cell)

`abb_driver` has no service here -- its `Dockerfile` is a deliberate `exit 1`
scaffold; see [`../../vendor_split/abb_driver/README.md`](../../vendor_split/abb_driver/README.md).

## Build

```bash
docker compose -f docker-compose.yml build piper_driver
# ur_driver / fanuc_driver are large (full ROS 2 desktop + Gazebo + MoveIt)
```

## Run

```bash
docker compose -f docker-compose.yml up -d piper_driver
```

Then start `arm_api2` from the demomotion Humble container:

```bash
docker exec -it arm_api2_cont_humble bash
ros2 launch arm_api2 moveit2_iface.launch.py robot_name:=piper
```

Containers are meant to stay up (idle `bash`) between launches.
