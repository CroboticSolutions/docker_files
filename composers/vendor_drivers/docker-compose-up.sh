#!/bin/bash

# Grant X11 access up front so RViz (spawned inside piper_driver_cont by the
# MoveIt launch files) can always open a window -- DISPLAY and
# /tmp/.X11-unix are already mounted into these containers in
# docker-compose.yml, but nothing authorizes them to actually use the host's
# X server without this. Scoped to local root processes, since these
# containers all run as root (no USER in their Dockerfiles), rather than the
# wider `xhost +local:docker`/`xhost +`. No-ops quietly if `xhost` isn't
# present (e.g. a Wayland-only or headless host).
xhost +si:localuser:root >/dev/null 2>&1 || true

# Run docker compose with any passed arguments, e.g.:
#   ./docker-compose-up.sh up -d arm_api2 piper_driver realsense
docker compose "$@"
