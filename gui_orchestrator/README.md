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

`demomotion_gui`, `ros2_dash_gui`, and `aiortc_webrtc_ros` are all private
repos, cloned over SSH at build time -- build with BuildKit and your
forwarded SSH agent (needs a key with read access to all three
`CroboticSolutions` repos; check with `ssh-add -l`). No local
`demomotion_gui` checkout is needed -- everything this Dockerfile needs
(itself + `entrypoint.sh`) lives in `docker_files`, so it's a single-context
build:

```bash
DOCKER_BUILDKIT=1 docker build --ssh default \
  -t gui_orchestrator \
  -f /home/martin/Crobotics/docker_files/gui_orchestrator/Dockerfile \
  /home/martin/Crobotics/docker_files/gui_orchestrator
```

The `ros2_dash_gui` (`mstigla/devel`), `arm_api2_msgs` (`apirsic/devel`), and
`demomotion_gui` (`merge/main-into-mstigla-compose-devel`) branches are
hardcoded directly in the `RUN git clone -b ...` lines, not build args --
edit the Dockerfile itself to point at a different branch, then rebuild
(`aiortc_webrtc_ros` has no branch pin, it just clones the repo's default).

## Run

```bash
docker run -d --restart unless-stopped \
  --name gui_orchestrator \
  --network host \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /home/martin/Crobotics/docker_files:/home/martin/Crobotics/docker_files \
  -e VENDOR_DRIVERS_COMPOSE_FILE=/home/martin/Crobotics/docker_files/composers/vendor_drivers/docker-compose.yml \
  gui_orchestrator
```

See "Restarting safely" below for why `-d --restart unless-stopped` is
preferred over `-it --rm` outside a quick foreground debug session.

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

**Container pull/build progress:** `_ensure_containers_up`'s `docker
pull`/`compose build`/`compose up -d` calls (run when a hardware profile's
vendor container isn't up yet) stream their output live into an in-memory
ring buffer instead of running fully captured, so a slow Hub pull isn't
silent. Exposed at `GET /status` as `container_setup: { lines, active }`;
the setup wizard's confirm step and `LaunchStatusPanel` both render it as a
collapsible terminal (`ContainerSetupTerminal` in
`src/pages/world-setup/LaunchStatusPanel.tsx`).

Every real-Piper hardware profile also ends with a `set_cartesian_default`
step (`hardware_stacks.py`'s `_cartesian_default_step`) a few seconds after
`arm_api2` comes up: it calls `/arm/change_state` with `CART_TRAJ_CTL`, the
same service the GUI's own StateControl "Cartesian Control" button calls --
so the arm lands in Cartesian mode automatically instead of sitting in
`IDLE` until someone clicks the button. `ARM_API2_CARTESIAN_DEFAULT_EXTRA_DELAY_S`
(default `8`) controls how long after `arm_api2`'s own launch delay it waits
before making that call.

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
- A container restart **does not pick up source changes** -- `demomotion_gui`
  (frontend + `launch_server`), `ros2_dash_gui`, `arm_api2_msgs`, and
  `aiortc_webrtc_ros` are all `git clone`'d at *build* time into
  `/root/arms_ws/src/<repo>` (one root for everything this image clones, even
  though `demomotion_gui`/`aiortc_webrtc_ros` aren't colcon packages and
  `colcon build` skips them), not bind-mounted. You need a `docker build`
  first. Note also that a plain rebuild can reuse Docker's cached layer for a
  `git clone` step even after new commits land on that repo's branch, since
  the `RUN git clone ...` command text itself hasn't changed -- if you only
  changed something in one of those repos (not this Dockerfile), rebuild with
  `--no-cache` or you'll silently get the old code.
- `launch_server`'s hardware-stack bookkeeping (`/status`'s
  `hardware_running`/`hardware_stack_id`) lives in this process's own
  memory. Restarting this container does **not** stop the real robot
  processes in the sibling driver containers (they keep running fine), but
  it does make `launch_server` forget it ever started them. Clicking "Launch
  Hardware" in the wizard in that state used to launch a second copy on top
  of the first (two sets of driver processes, e.g. duplicate
  `piper_read_slave_joint`/`piper_ctrl_single_node`, racing each other over
  the same physical hardware) -- each `ros2 launch` step is now wrapped in a
  non-blocking `flock` keyed by its step name (`_flock_wrap_ros2_launch` in
  `launch_server/main.py`), so a second launch attempt for a step that's
  still alive fails immediately instead of duplicating. The lock file lives
  inside the *driver* container the step runs in (e.g.
  `/tmp/arm_api2_gui_launch_arm_api2.lock` in `arm_api2_cont`), not in
  this container, so it survives a `gui_orchestrator` restart even though
  `launch_server`'s own memory doesn't. Still worth doing one manual
  `/hardware/stop` + `/hardware/launch` cycle (or the wizard's stop/relaunch
  controls) after a restart to resync `/status` before trusting the UI.

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
