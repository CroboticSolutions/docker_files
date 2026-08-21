# abb_driver — SCAFFOLD ONLY, not verified. See README.md before building.
#
# This workspace does NOT have a working ABB ROS 2 driver. What exists locally
# is config only:
#   - /root/abb_irb6700_145_320_gripper_moveit_config  (MoveIt config/URDF for
#     simulation, no live hardware driver)
#   - arm_api2's config/abb/*.yaml                     (kinematics config for
#     arm_api2's "abb" robot_name, no live driver either)
#
# The obvious upstream candidate, ros-industrial/abb_robot_driver, is ROS 1
# (catkin, Melodic/Bionic/Focal) and will NOT build in this ROS 2 Jazzy stack.
# No maintained ROS 2 equivalent was found. Do not `docker build` this file
# as-is expecting a working driver — fill in the TODO clone step below once
# you've picked (or written) an actual ROS 2 ABB driver, then delete this
# banner. See README.md for what to check before picking one.
#
# Everything above the TODO (base OS, ROS 2, MoveIt, ros2_control, Gazebo) is
# the same proven layer used by ur_driver / piper_driver / fanuc_driver, so it
# should not need changes.

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
      wget && \
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

# Gazebo + ROS–Gazebo bridges — only needed if the eventual driver simulates
# via Gazebo; drop this block if it doesn't.
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

WORKDIR /root/abb_ws/src

# TODO: replace with the actual ABB ROS 2 driver repo(s)/branch(es) you intend
# to use, e.g.:
#   RUN git clone -b <branch> https://github.com/<org>/<abb_ros2_driver>.git
# Bring the existing local config along either way — copy it in (see README):
#   /root/abb_irb6700_145_320_gripper_moveit_config
#   arm_api2's config/abb/*.yaml (already in the arm_api2 container, not here)
RUN echo "No ABB driver repo configured yet — see the banner at the top of this Dockerfile." && \
    exit 1

WORKDIR /root/abb_ws

RUN apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
      empy \
      catkin_pkg \
      lark

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install

RUN echo "" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/abb_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]
