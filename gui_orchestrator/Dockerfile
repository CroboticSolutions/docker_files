# syntax=docker/dockerfile:1
#
# gui_orchestrator -- demomotion_gui frontend + launch_server (setup wizard,
# FastAPI) + the ros2_dash_gui websocket bridge + the two aiortc_webrtc_ros
# WebRTC video servers (raw + SAM-annotated camera feeds), in one container.
#
# What's NOT in here: arm_api2, vendor driver (PiPER/RealSense), simulation,
# and the perception (HaMeR/WiLoR hand-pose) containers -- those stay as
# separate sibling containers on the host (see docker_files/composers/). This
# container's bridge.py talks to them over the shared host-network ROS graph,
# and launch_server's wizard controls them via `docker compose`/`docker exec`
# through the host's Docker socket, which must be bind-mounted in at *run*
# time -- see README.md next to this file for the full run command; this
# image alone is not runnable as a complete stack.
#
# Build (demomotion_gui and ros2_dash_gui are both private repos, cloned over
# SSH -- needs a key with read access loaded in your agent, see `ssh-add -l`).
# No local demomotion_gui checkout needed as build context -- everything this
# Dockerfile needs (itself + entrypoint.sh) lives in docker_files:
#   DOCKER_BUILDKIT=1 docker build --ssh default \
#     -t gui_orchestrator \
#     -f /home/martin/Crobotics/docker_files/gui_orchestrator/Dockerfile \
#     /home/martin/Crobotics/docker_files/gui_orchestrator

FROM ubuntu:noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS2_DISTRO=jazzy
ARG NODE_MAJOR=24

ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=${ROS2_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Zagreb \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# libgl1: bridge.py's `import open3d` dlopens libGL.so.1 at import time even
# when running headless -- without it the bridge process crash-loops.
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo "${TZ}" > /etc/timezone && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      curl \
      git \
      gnupg \
      libgl1 \
      lsb-release \
      openssh-client \
      python3-pip \
      sudo \
      wget && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ros-base + Cyclone DDS + the ROS packages bridge.py imports directly
# (tf2_ros/tf2_sensor_msgs for the point-cloud transform, sensor_msgs_py for
# read_points -- confirmed by reading scripts/bridge.py's own imports).
# cv-bridge is for aiortc_webrtc_ros's server_ros.py (sensor_msgs/Image ->
# OpenCV ndarray); python3-opencv for its own direct `import cv2`. Both are
# apt packages with compiled extensions built against NumPy 1.x's C ABI --
# see the `numpy<2` pin below. No MoveIt/ros2_control/RViz -- those belong to
# arm_api2 / the driver containers, not this one.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-ros-base \
      ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS2_DISTRO}-tf2-ros \
      ros-${ROS2_DISTRO}-tf2-sensor-msgs \
      ros-${ROS2_DISTRO}-sensor-msgs-py \
      ros-${ROS2_DISTRO}-cv-bridge \
      python3-opencv \
      python3-colcon-common-extensions \
      python3-vcstool \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Docker CLI + compose plugin -- launch_server's setup wizard shells out to
# `docker compose`/`docker exec`/`docker ps` to control the vendor-driver
# containers (PiPER/RealSense); see launch_server/hardware_stacks.py and
# main.py's `_run_compose_step`/`_docker_exec_ros2_args`. Those calls only do
# anything useful if /var/run/docker.sock from the host is bind-mounted in at
# run time (Docker-out-of-Docker) -- see README.md.
RUN install -m 0755 -d /etc/apt/keyrings && \
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc && \
    chmod a+r /etc/apt/keyrings/docker.asc && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] \
      https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") stable" \
      | tee /etc/apt/sources.list.d/docker.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends docker-ce-cli docker-compose-plugin && \
    rm -rf /var/lib/apt/lists/*

# Node.js -- builds the Vite/React frontend. It's served by launch_server
# itself (main.py mounts dist/ via SpaStaticFiles), so no separate web server
# is needed at runtime.
RUN curl -fsSL https://deb.nodesource.com/setup_${NODE_MAJOR}.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    rm -rf /var/lib/apt/lists/*

# ros2_dash_gui is a private repo -- clone over SSH using the host's forwarded
# agent (`docker build --ssh default ...`), not HTTPS.
RUN mkdir -p -m 0700 /root/.ssh && \
    ssh-keyscan -t rsa,ecdsa,ed25519 github.com >> /root/.ssh/known_hosts

WORKDIR /root/arms_ws/src
RUN --mount=type=ssh git clone -b mstigla/devel \
      git@github.com:CroboticSolutions/ros2_dash_gui.git

# arm_api2_msgs -- bridge.py imports these message types by name to
# (de)serialize the topics it bridges; it's a public repo (HTTPS, no SSH
# needed). Just the interfaces, not arm_api2 itself -- this container never
# runs arm_api2, only talks to it over ROS topics. apirsic/devel to match the
# branch arm_api2/arm_api2_msgs are built from elsewhere in this project (see
# docker_files/vendor_split/arm_api2/Dockerfile) -- main is missing messages
# (e.g. PlanStatus) that bridge.py expects.
RUN git clone -b apirsic/devel https://github.com/CroboticSolutions/arm_api2_msgs.git

# aiortc_webrtc_ros -- CroboticSolutions' fork of aiortc with
# examples/server/server_ros.py added (bridges a ROS Image/CompressedImage
# topic to a WebRTC video track). Private repo, cloned over SSH like
# ros2_dash_gui. Not a colcon package (no package.xml), so colcon build below
# skips it, but it lives under arms_ws/src anyway to keep every repo this
# image clones under one root instead of scattered across /root.
RUN --mount=type=ssh git clone git@github.com:CroboticSolutions/aiortc_webrtc_ros.git \
      /root/arms_ws/src/aiortc_webrtc_ros

# aiohttp isn't declared in aiortc_webrtc_ros's own pyproject.toml (only
# under its `dev` extra, meant for developing aiortc itself) but
# examples/server/server_ros.py needs it directly. `pip install .../...`
# installs the aiortc package itself (aioice/av/cryptography/pyee/etc, per
# that pyproject.toml's `dependencies`).
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
      aiohttp \
      /root/arms_ws/src/aiortc_webrtc_ros

WORKDIR /root/arms_ws

RUN apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# bridge.py's own pip deps (numpy/open3d/pyyaml/websockets), confirmed by
# reading scripts/bridge.py's imports directly. dash_gui.py's deps (dash,
# opencv-python, google-genai, ollama, gdown) are deliberately NOT installed
# -- this container never runs the Dash frontend, demomotion_gui's React/Vite
# app replaces it.
# --ignore-installed: open3d pulls in psutil, which is already present via
# apt (a ros-dev-tools/rosdep dependency) without pip RECORD metadata, so a
# plain install fails trying to uninstall it first -- shadow it instead.
# numpy<2: ros-jazzy-cv-bridge's compiled cv_bridge_boost extension (and
# python3-opencv) are apt packages built against NumPy 1.x's C ABI -- an
# unpinned numpy here resolves to 2.x and both ImportError at runtime
# ("compiled using NumPy 1.x cannot be run in NumPy 2.x").
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
      "numpy<2" \
      open3d \
      pyyaml \
      websockets

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install

# --- demomotion_gui: frontend (built, served by launch_server) + backend ---
# Private repo, cloned over SSH like ros2_dash_gui/aiortc_webrtc_ros above --
# no local checkout needed as build context anymore. Not a colcon package
# either (npm frontend + FastAPI backend, no package.xml), but lives under
# arms_ws/src for the same one-root-for-everything reason as aiortc_webrtc_ros.

WORKDIR /root/arms_ws/src/demomotion_gui

RUN --mount=type=ssh git clone -b merge/main-into-mstigla-compose-devel \
      git@github.com:CroboticSolutions/demomotion_gui.git .

RUN npm install --legacy-peer-deps

RUN python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed -r launch_server/requirements.txt

RUN npm run build

RUN echo "" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/arms_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENV ARM_API2_LAUNCH_LOG_DIR=/tmp/arm_api2_gui_launch_logs

EXPOSE 8080 9094 8181 8182

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
