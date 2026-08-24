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

## Restarting safely

`entrypoint.sh` starts 4 background processes (the `ros2_dash_gui` bridge,
`launch_server`'s uvicorn, and the two `aiortc_webrtc_ros` servers) and does
`wait -n` on all of them: **the instant any one of the four exits, for any
reason, the entrypoint tears the other three down and the container itself
exits.** This is deliberate (fail-fast instead of limping along with one
dead component), but it means:

- **Never `docker exec gui_orchestrator kill <pid>` on one of those four
  child processes** (e.g. to bounce just the bridge after a `bridge.py`
  code change) -- that takes the *whole* container down, not just that one
  process. If you ran it with `--rm` (see `## Run` above), the container is
  then gone, not just stopped, and you'll need to re-`docker run` it.
- To restart everything cleanly, use `docker restart gui_orchestrator` (or
  `docker stop` + re-`docker run` if you used `--rm`) -- that sends `SIGTERM`
  to the entrypoint script itself, which runs its own `cleanup()` trap
  correctly instead of racing `wait -n`.
- Prefer running with `--restart unless-stopped` (no `--rm`) over `-it --rm`
  for anything other than a quick foreground debug session -- `--rm` means
  any single child dying (a crash, an OOM, an accidental `kill`) silently
  deletes the container instead of leaving something to `docker start`/
  `docker logs` after the fact.
- A container restart **does not pick up source changes** -- both
  `demomotion_gui` (frontend + `launch_server`) and `ros2_dash_gui` are
  `COPY`'d/`git clone`'d at *build* time, not bind-mounted. You need a
  `docker build` first. Note also that a plain rebuild can reuse Docker's
  cached layer for the `ros2_dash_gui` `git clone` step even after new
  commits land on its branch, since the `RUN git clone ...` command text
  itself hasn't changed -- if you only changed something in that repo (not
  this Dockerfile or `demomotion_gui`), rebuild with `--no-cache` or you'll
  silently get the old bridge code.
- `launch_server`'s hardware-stack bookkeeping (`/status`'s
  `hardware_running`/`hardware_stack_id`) lives in this process's own
  memory. Restarting this container does **not** stop the real robot
  processes in the sibling driver containers (they keep running fine), but
  it does make `launch_server` forget it ever started them. Don't click
  "Launch Hardware" in the wizard in that state -- since it thinks nothing
  is running, it will launch a second copy on top of the first instead of
  replacing it, leaving two sets of driver processes (e.g. duplicate
  `piper_read_slave_joint`/`piper_ctrl_single_node`) racing each other over
  the same physical hardware. Do one manual `/hardware/stop` +
  `/hardware/launch` cycle (or the wizard's stop/relaunch controls) to
  resync state before trusting the UI again.

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
