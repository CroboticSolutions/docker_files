#!/bin/bash
# Starts the ros2_dash_gui bridge and launch_server (which also serves the
# built frontend) as children of this shell, so a `docker stop` (SIGTERM)
# reaches both instead of only whichever process happens to be PID 1.
set -eo pipefail

# ROS's setup.bash references optional env vars (e.g. AMENT_TRACE_SETUP_FILES)
# without defaults, so it isn't nounset-safe -- source with -u off, then
# re-enable it for the rest of this script.
source /opt/ros/"${ROS_DISTRO}"/setup.bash
source /root/arms_ws/install/setup.bash

set -u

BRIDGE_WS_PORT="${BRIDGE_WS_PORT:-9094}"
ROBOT_NAMESPACE_PREFIX="${ROBOT_NAMESPACE_PREFIX:-}"
LAUNCH_SERVER_HOST="${LAUNCH_SERVER_HOST:-0.0.0.0}"
LAUNCH_SERVER_PORT="${LAUNCH_SERVER_PORT:-8080}"
# Must match the frontend's build-time defaults (CameraStream.tsx's
# VITE_WEBRTC_RAW_PORT/VITE_WEBRTC_ANNOTATED_PORT, both baked into dist/ at
# image build time) unless those were overridden -- see README.md.
WEBRTC_RAW_PORT="${WEBRTC_RAW_PORT:-8181}"
WEBRTC_ANNOTATED_PORT="${WEBRTC_ANNOTATED_PORT:-8182}"
WEBRTC_RAW_IMAGE_TOPIC="${WEBRTC_RAW_IMAGE_TOPIC:-/piper/camera/image_raw}"
WEBRTC_ANNOTATED_IMAGE_TOPIC="${WEBRTC_ANNOTATED_IMAGE_TOPIC:-/sam/annotated_image}"

pids=()

cleanup() {
    trap - TERM INT
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup TERM INT

# robot_namespace_prefix must be empty for Piper/root-namespace arm_api2, or
# e.g. "ur" for the UR multi-robot sim -- see ros2_dash_gui/README.md. `ros2
# launch` itself rejects a `name:=<empty>` argument outright, so only pass it
# when non-empty and let the launch file's own default (root namespace) apply.
launch_args=("websocket_port:=${BRIDGE_WS_PORT}")
if [ -n "${ROBOT_NAMESPACE_PREFIX}" ]; then
    launch_args+=("robot_namespace_prefix:=${ROBOT_NAMESPACE_PREFIX}")
fi
ros2 launch ros2_dash_gui bridge_piper.launch.py "${launch_args[@]}" &
pids+=("$!")

cd /opt/demomotion_gui/launch_server
uvicorn main:app --host "${LAUNCH_SERVER_HOST}" --port "${LAUNCH_SERVER_PORT}" --workers 1 &
pids+=("$!")

# Two aiortc_webrtc_ros instances -- CameraStream.tsx opens one WebRTC peer
# connection per port (raw camera, SAM-annotated overlay), each its own
# server_ros.py process since the aiortc/rclpy globals in that script are
# module-level, not designed to serve more than one video track per process.
ROS_IMAGE_TOPIC="${WEBRTC_RAW_IMAGE_TOPIC}" \
    python3 /opt/aiortc_webrtc_ros/examples/server/server_ros.py --port "${WEBRTC_RAW_PORT}" &
pids+=("$!")
ROS_IMAGE_TOPIC="${WEBRTC_ANNOTATED_IMAGE_TOPIC}" \
    python3 /opt/aiortc_webrtc_ros/examples/server/server_ros.py --port "${WEBRTC_ANNOTATED_PORT}" &
pids+=("$!")

# Exit as soon as either process dies, taking the other down with it.
wait -n "${pids[@]}"
exit_code=$?
cleanup
exit "$exit_code"
