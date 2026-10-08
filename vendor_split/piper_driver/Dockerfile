# piper_driver — AgileX Piper ROS 2 driver (CAN) + Gazebo simulation + MoveIt bring-up.
#
# Runs at ROS namespace root by convention (no per-arm namespace prefix), matching
# how the GUI treats Piper as the "Arm" / workspace-root robot — see ../README.md.
#
# Build:  docker build -t piper_driver:latest .
# Run:    see README.md in this folder — real hardware needs CAN access
#         (--privileged or --cap-add=NET_ADMIN, plus the CAN adapter under /dev).

FROM ubuntu:noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS2_DISTRO=jazzy

ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_DISTRO=${ROS2_DISTRO} \
    ROS_DISTRO=${ROS2_DISTRO} \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Zagreb \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo "${TZ}" > /etc/timezone && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      curl \
      git \
      gnupg \
      libegl1 \
      libgl1 \
      libglu1-mesa \
      lsb-release \
      python3-pip \
      sudo \
      wget \
      can-utils \
      ethtool \
      iproute2 && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-desktop \
      ros-${ROS2_DISTRO}-moveit \
      ros-${ROS2_DISTRO}-moveit-visual-tools \
      ros-${ROS2_DISTRO}-moveit-servo \
      ros-${ROS2_DISTRO}-ros2-control \
      ros-${ROS2_DISTRO}-ros2-controllers \
      ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions \
      python3-vcstool \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Gazebo + ROS–Gazebo bridges (piper_gazebo / piper_sim_moveit)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends gz-ionic && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-ros-gz-sim \
      ros-${ROS2_DISTRO}-ros-gz-bridge || \
    apt-get install -y --no-install-recommends ros-${ROS2_DISTRO}-ros-gz || true && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/piper_ws/src

# Every CroboticSolutions repo's tested version lives on `main` (since 2026-10-05).
RUN git clone -b main https://github.com/CroboticSolutions/piper_ros.git

# piper_welding_gun.urdf.xacro with wrist_camera:=femto_bolt (the Femto Bolt
# welding-gun profile) loads package://orbbec_description meshes, so MoveIt
# here needs that package even though the driver itself runs in orbbec_cont.
# Only the description is built: the driver packages are COLCON_IGNOREd, which
# also keeps rosdep from pulling the Orbbec SDK's dependencies into this image.
RUN git clone --depth 1 -b v2-main https://github.com/CroboticSolutions/OrbbecSDK_ROS2.git && \
    touch OrbbecSDK_ROS2/orbbec_camera/COLCON_IGNORE OrbbecSDK_ROS2/orbbec_camera_msgs/COLCON_IGNORE

WORKDIR /root/piper_ws

# rosdep init writes /etc/ros/rosdep/sources.list.d/20-default.list pointing every
# source at .../rosdistro/master/..., and GitHub's raw-content edge has been
# returning a hard "400: Bad Request" specifically for the master-branch-alias
# request to rosdep/base.yaml (other files under the same master ref, and base.yaml
# itself fetched by a pinned commit SHA, both resolve fine) -- so re-point that file
# at the current master commit SHA instead of the branch alias before updating.
RUN apt-get update && \
    rosdep init || true && \
    ROSDISTRO_SHA=$(curl -sS https://api.github.com/repos/ros/rosdistro/commits/master | grep -m1 '"sha"' | sed -E 's/.*"sha": *"([a-f0-9]+)".*/\1/') && \
    [ -n "$ROSDISTRO_SHA" ] && \
    sed -i "s#/rosdistro/master/#/rosdistro/${ROSDISTRO_SHA}/#g" /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# piper_ros's Python nodes (piper_ctrl_single_node.py etc.) talk to the CAN bus
# through AgileX's piper_sdk, not through rosdep/apt.
RUN pip3 install --no-cache-dir --break-system-packages \
      empy \
      catkin_pkg \
      lark \
      piper_sdk

# piper_ros main defaults piper_ctrl_single_node to sdk_backend:=pyagxarm
# (piper/sdk_backend.py); piper_sdk stays as the fallback. Pinned to the commit
# validated on the robot (pyAgxArm 1.0.0) -- upgrades must pass
# test_sdk_backend_frames.py.
RUN pip3 install --no-cache-dir --break-system-packages \
      "git+https://github.com/agilexrobotics/pyAgxArm.git@841a625f5f4920e776f20b934eb13048b747e6d0"

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install

RUN echo "" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/piper_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]
