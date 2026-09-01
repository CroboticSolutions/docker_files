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

COMPOSE_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/docker-compose.yml"

# service -> container_name, hardcoded to match docker-compose.yml exactly.
declare -A CONTAINER_NAME=(
  [arm_api2]=arm_api2_cont
  [piper_driver]=piper_driver_cont
  [realsense]=realsense_cont
  [ur_driver]=ur_driver_cont
  [fanuc_driver]=fanuc_driver_cont
  [perception]=perception_cont
)

# service -> "hub_image local_image", only for services with a published
# Docker Hub image (mirrors CONTAINER_HUB_IMAGE in
# demomotion_gui/launch_server/hardware_stacks.py -- ur_driver/fanuc_driver
# have no Hub image yet, so they always fall back to a local build).
declare -A HUB_IMAGE=(
  [arm_api2]="croboticsolutions/arm_api2_img:jazzy arm_api2_img:jazzy"
  [piper_driver]="croboticsolutions/piper_driver_img:jazzy piper_driver_img:jazzy"
  [realsense]="croboticsolutions/realsense_img:jazzy realsense_img:jazzy"
  [perception]="croboticsolutions/perception_img:jazzy perception_img:jazzy"
)

# Dev-friendly `up`: a container already running under its expected name is
# left alone (no recreate, so no "name already in use" conflict); one that
# exists but is stopped gets a plain `docker start` instead of being routed
# through `docker compose up -d` (which would try to *create* a container
# under that name and get a conflict, since this container isn't tracked by
# compose); only a container that doesn't exist at all gets its image pulled
# from Docker Hub and retagged, falling back to `docker compose build` only
# if there's no Hub image or the pull fails. Same order launch_server/
# main.py's `_ensure_containers_up` already uses for the wizard's
# auto-start, so manual runs behave the same way.
smart_up() {
  local services=("$@")
  if [ ${#services[@]} -eq 0 ]; then
    services=("${!CONTAINER_NAME[@]}")
  fi

  local running existing
  running="$(docker ps --format '{{.Names}}')"
  existing="$(docker ps -a --format '{{.Names}}')"

  local not_running=()
  local svc name
  for svc in "${services[@]}"; do
    name="${CONTAINER_NAME[$svc]:-}"
    if [ -z "$name" ]; then
      echo "unknown service: $svc" >&2
      exit 1
    fi
    if grep -qx "$name" <<<"$running"; then
      echo "== $name already running, leaving it as-is =="
    else
      not_running+=("$svc")
    fi
  done

  if [ ${#not_running[@]} -eq 0 ]; then
    echo "Nothing to start -- all requested containers already running."
    return
  fi

  local stopped=()
  local missing=()
  for svc in "${not_running[@]}"; do
    name="${CONTAINER_NAME[$svc]}"
    if grep -qx "$name" <<<"$existing"; then
      stopped+=("$name")
    else
      missing+=("$svc")
    fi
  done

  if [ ${#stopped[@]} -gt 0 ]; then
    echo "== starting stopped containers: ${stopped[*]} =="
    docker start "${stopped[@]}"
  fi

  if [ ${#missing[@]} -eq 0 ]; then
    return
  fi

  local to_build=()
  local hub hub_image local_image
  for svc in "${missing[@]}"; do
    hub="${HUB_IMAGE[$svc]:-}"
    if [ -z "$hub" ]; then
      to_build+=("$svc")
      continue
    fi
    read -r hub_image local_image <<<"$hub"
    if docker image inspect "$local_image" >/dev/null 2>&1; then
      echo "== $local_image already present locally =="
      continue
    fi
    echo "== pulling $hub_image -> $local_image =="
    if docker pull "$hub_image" && docker tag "$hub_image" "$local_image"; then
      continue
    fi
    echo "== pull failed for $hub_image, falling back to local build =="
    to_build+=("$svc")
  done

  if [ ${#to_build[@]} -gt 0 ]; then
    echo "== building: ${to_build[*]} =="
    docker compose -f "$COMPOSE_FILE" build "${to_build[@]}"
  fi

  echo "== docker compose up -d: ${missing[*]} =="
  docker compose -f "$COMPOSE_FILE" up -d "${missing[@]}"
}

# Accept (and discard) an explicit `-f <file>` so old invocations still work,
# e.g. `./docker-compose-up.sh -f docker-compose.yml up -d perception`.
if [ "${1:-}" = "-f" ]; then
  COMPOSE_FILE="$2"
  shift 2
fi

# `./docker-compose-up.sh up [-d] [service ...]` gets the smart dev-friendly
# treatment above; any other docker compose subcommand (build, down, logs,
# ...) passes straight through.
if [ "${1:-}" = "up" ]; then
  shift
  services=()
  for a in "$@"; do
    [ "$a" = "-d" ] && continue
    services+=("$a")
  done
  smart_up "${services[@]}"
else
  docker compose -f "$COMPOSE_FILE" "$@"
fi