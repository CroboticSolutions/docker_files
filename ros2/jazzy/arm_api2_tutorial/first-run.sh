#!/usr/bin/env bash
# Build the arm_api2:tutorial image and start an interactive shell. See --help.

set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-arm_api2:tutorial}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

BUILD=1
RUN_X11=0
NET_HOST=0
CONTAINER_NAME=""

usage() {
  cat <<'EOF'
Usage: first-run.sh [options]

  (default)     docker build, then interactive bash -l in the container
  --no-build    skip build; run existing image
  --x11, --gui  mount X11 socket and DISPLAY (Gazebo / RViz on Linux)
  --net-host    docker --network host (often helps ROS 2 DDS between host and container)
  --name NAME   container name (disables --rm)
  -h, --help    show this help

Environment:
  IMAGE_NAME    image name:tag (default: arm_api2:tutorial)
EOF
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0 ;;
    --x11|--gui) RUN_X11=1 ;;
    --net-host) NET_HOST=1 ;;
    --name)
      shift
      [[ $# -gt 0 ]] || { echo "error: --name requires a value" >&2; exit 1; }
      CONTAINER_NAME="$1"
      ;;
    -h|--help) usage 0 ;;
    *)
      echo "error: unknown option: $1" >&2
      usage 1
      ;;
  esac
  shift
done

if ! command -v docker >/dev/null 2>&1; then
  echo "error: docker not found in PATH" >&2
  exit 1
fi

if [[ ! -f "$DOCKERFILE" ]]; then
  echo "error: Dockerfile not found: $DOCKERFILE" >&2
  exit 1
fi

if [[ "$BUILD" -eq 1 ]]; then
  echo "Building image: $IMAGE_NAME"
  docker build -t "$IMAGE_NAME" -f "$DOCKERFILE" "$SCRIPT_DIR"
fi

RUN_ARGS=(-it)
if [[ -z "$CONTAINER_NAME" ]]; then
  RUN_ARGS+=(--rm)
else
  RUN_ARGS+=(--name "$CONTAINER_NAME")
fi

if [[ "$NET_HOST" -eq 1 ]]; then
  RUN_ARGS+=(--network host)
fi

if [[ "$RUN_X11" -eq 1 ]]; then
  RUN_ARGS+=(-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  echo "note: for X11 you may need on the host: xhost +local:root" >&2
fi

echo "Starting container: $IMAGE_NAME (bash -l)"
exec docker run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -l
