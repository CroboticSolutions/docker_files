# Camera Test Compose

Single Compose file with one service per camera driver. Originally used to
smoke-test that the demomotion_gui setup wizard could start a camera
container via a `POST /camera/docker/*` test hook; that hook (and its
buttons in the wizard) has since been removed. This compose file itself is
standalone (not called by `launch_server`) -- useful for manually building/
running a camera container in isolation.

**realsense is now also containerized for real wizard launches** -- see
[`../vendor_drivers`](../vendor_drivers), whose `realsense` service is the
one `hardware_stacks.py`/`launch_server` actually starts for the PiPER +
RealSense D435 profile (same image/build context as the `realsense` service
here, just wired into the wizard's `docker exec` launch path). The
`luxonis` service below is not yet wired the same way -- the wizard's OAK-D
Pro W profile still runs `depthai_ros_driver_v3`'s `rgbd_pro_w_pcl.launch.py`
directly wherever `launch_server` runs, since that's a different
package/launch file than the OAK-D SR-only `depthai_ros_driver` workspace
`../../luxonis` builds.

Services:
- **realsense** -- [../../realsense](../../realsense) (Intel RealSense D435/D400 series)
- **luxonis** -- [../../luxonis](../../luxonis) (Luxonis OAK-D SR)

## Build

```bash
docker compose -f docker-compose.yml build realsense
docker compose -f docker-compose.yml build luxonis
```

## Run one service

```bash
docker compose -f docker-compose.yml up -d realsense
# or
docker compose -f docker-compose.yml up -d luxonis
```

## Stop

```bash
docker compose -f docker-compose.yml stop realsense
docker compose -f docker-compose.yml stop luxonis
```

