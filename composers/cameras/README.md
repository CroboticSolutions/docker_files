# Camera Test Compose

Single Compose file with one service per camera driver. Originally used to
smoke-test that the demomotion_gui setup wizard could start a camera
container via a `POST /camera/docker/*` test hook; that hook (and its
buttons in the wizard) has since been removed -- the wizard's real camera
launches still run `ros2 launch realsense2_camera` / `depthai_ros_driver_v3`
directly wherever `launch_server` runs, unchanged. This compose file is now
standalone (not called by `launch_server`) -- useful for manually building/
running a camera container, or as a starting point if camera launches get
containerized later, similar to [`../vendor_drivers`](../vendor_drivers).

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

