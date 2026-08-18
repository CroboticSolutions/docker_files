# Camera Test Compose

Single Compose file with one service per camera driver, used to smoke-test that
the demomotion_gui setup wizard can actually start a camera container (as
opposed to assuming `ros2` is already sourced on the host).

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

This is what `demomotion_gui/launch_server`'s `POST /camera/docker/launch` and
`POST /camera/docker/stop` endpoints shell out to -- see
[../../../demomotion_gui/launch_server/main.py](../../../demomotion_gui/launch_server/main.py).
