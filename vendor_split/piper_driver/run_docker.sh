#!/bin/bash

CONTAINER_NAME=piper_driver
IMAGE_NAME=piper_driver:latest

# Grant the container's X11 access up front so RViz (spawned by the MoveIt
# launch files run inside this container) can always open a window without
# a manual `xhost` call first. Scoped to local root processes -- the
# container runs as root (no USER in the Dockerfile) -- rather than the
# wide-open `xhost +local:docker`/`xhost +`. No-ops quietly if `xhost` isn't
# present (e.g. a Wayland-only or headless host).
xhost +si:localuser:root >/dev/null 2>&1 || true

# Real hardware needs --privileged + /dev for the USB-CAN adapter (can0).
# Drop --privileged/-v /dev:/dev for simulation-only use.
docker run \
  -it \
  --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
