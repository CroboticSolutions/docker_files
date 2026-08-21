# gui_orchestrator

One container for the `demomotion_gui` frontend, its `launch_server` FastAPI
backend (the hardware setup wizard), the `ros2_dash_gui` websocket bridge,
and two `aiortc_webrtc_ros` WebRTC video servers (raw + SAM-annotated camera
feeds -- see `CameraStream.tsx`). Everything else -- `arm_api2`, the
PiPER/RealSense vendor drivers, Gazebo, the `perception` (HaMeR/WiLoR
hand-pose) container -- stays as a separate sibling container on the host
(see `docker_files/composers/`). This image is meant to sit alongside them,
not replace them.

## Why this needs the host's Docker socket

`launch_server`'s setup wizard shells out to `docker compose`/`docker exec`/
`docker ps` directly (see `launch_server/hardware_stacks.py` and
`main.py`'s `_run_compose_step`/`_docker_exec_ros2_args`) to bring up and
control the PiPER/RealSense containers defined in
`docker_files/composers/vendor_drivers/docker-compose.yml`. Those calls only
do anything if this container can reach the *host's* Docker daemon, so it
needs `/var/run/docker.sock` bind-mounted in (Docker-out-of-Docker) -- a
nested/isolated dockerd would see none of the containers already running on
the host and the wizard would have nothing to control.

**Security note:** mounting the host's Docker socket into a container is
equivalent to giving that container root on the host -- there's no privilege
boundary between "can talk to the socket" and "can run `docker run -v
/:/host ... chroot /host`". Acceptable for an internal dev/demo box that
already runs an unauthenticated ROS bridge on the LAN, but worth knowing
before you expose this container's ports beyond that.

## Build

`ros2_dash_gui` and `aiortc_webrtc_ros` are both private repos, cloned over
SSH at build time -- build with BuildKit and your forwarded SSH agent (needs
a key with read access to both `CroboticSolutions/ros2_dash_gui` and
`CroboticSolutions/aiortc_webrtc_ros`; check with `ssh-add -l`). This Dockerfile
lives in `docker_files`, but the frontend/launch_server source it `COPY`s is
in `demomotion_gui`, so the build uses two contexts: the main one is
`demomotion_gui`, plus a named `orchestrator` context pointing back at this
directory for `entrypoint.sh`:

```bash
DOCKER_BUILDKIT=1 docker build --ssh default \
  --build-context orchestrator=/home/martin/Crobotics/docker_files/gui_orchestrator \
  -t gui_orchestrator \
  -f /home/martin/Crobotics/docker_files/gui_orchestrator/Dockerfile \
  /home/martin/Crobotics/demomotion_gui
```

## Run

```bash
docker run -it --rm \
  --name gui_orchestrator \
  --network host \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /home/martin/Crobotics/docker_files:/home/martin/Crobotics/docker_files \
  -e VENDOR_DRIVERS_COMPOSE_FILE=/home/martin/Crobotics/docker_files/composers/vendor_drivers/docker-compose.yml \
  gui_orchestrator
```

- `--network host` -- matches every other container in this project (see
  `docker_files/composers/demomotion/docker-compose.yml`); ROS 2 DDS
  discovery relies on host-network multicast, and with host networking
  there's no isolated network to bridge to begin with.
- The `docker_files` bind mount must land at the **same absolute path** it
  already lives at on the host. `docker compose`/`docker exec`, invoked
  through the socket-mounted CLI, are executed by the *host's* daemon, so
  relative build contexts and any bind-mounts declared inside those compose
  files resolve against the host filesystem, not this container's -- mount
  the whole tree at its real host path rather than somewhere convenient
  inside the container.
- `VENDOR_DRIVERS_COMPOSE_FILE` overrides `launch_server`'s default path
  lookup (`hardware_stacks.py`) -- only needed if your `docker_files`
  checkout isn't at the path baked into that default.

Frontend + wizard: `http://localhost:8080`. Bridge websocket:
`ws://localhost:9094`. WebRTC video: `http://localhost:8181/offer` (raw),
`http://localhost:8182/offer` (SAM-annotated) -- matches
`CameraStream.tsx`'s `VITE_WEBRTC_RAW_PORT`/`VITE_WEBRTC_ANNOTATED_PORT`
build-time defaults, so no frontend rebuild is needed unless those were
overridden.

## Env vars

| Var | Default | What |
|---|---|---|
| `BRIDGE_WS_PORT` | `9094` | `ros2_dash_gui` bridge websocket port |
| `ROBOT_NAMESPACE_PREFIX` | `` (empty) | Empty for Piper/root-namespace `arm_api2`; e.g. `ur` for the UR multi-robot sim |
| `LAUNCH_SERVER_PORT` | `8080` | `launch_server`/frontend port |
| `VENDOR_DRIVERS_COMPOSE_FILE` | `launch_server`'s built-in path | Path to `docker_files/composers/vendor_drivers/docker-compose.yml`, as seen from inside this container |
| `WEBRTC_RAW_PORT` | `8181` | Raw-camera `aiortc_webrtc_ros` server port |
| `WEBRTC_ANNOTATED_PORT` | `8182` | SAM-annotated `aiortc_webrtc_ros` server port |
| `WEBRTC_RAW_IMAGE_TOPIC` | `/piper/camera/image_raw` | Fallback ROS topic for the raw server, only used if a client's WebRTC offer omits `ros_image_topic` -- `CameraStream.tsx` always sends one, so this rarely applies |
| `WEBRTC_ANNOTATED_IMAGE_TOPIC` | `/sam/annotated_image` | Same fallback, for the annotated server |

## What's deliberately not in this image

- **arm_api2 / vendor drivers / Gazebo / `perception`** -- these are the
  containers the wizard *controls* (or, for `perception`, that
  `welding_stack.py`/`main.py` `docker exec` into), not containers this one
  runs itself.
- **`dash_gui.py`** and its pip deps (`dash`, `opencv-python`,
  `google-genai`, `ollama`, `gdown`) -- `ros2_dash_gui` ships both a Dash
  frontend and the bridge; this image only builds/runs the bridge
  (`bridge.py`), since `demomotion_gui`'s React/Vite app is the actual
  frontend here.
