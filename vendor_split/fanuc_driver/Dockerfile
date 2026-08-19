# fanuc_driver — FANUC streaming ros2_control driver (classic Fanuc + CRX cobots)
# + Gazebo simulation + MoveIt bring-up.
#
# Bundles four repos that are used together in the combined dev container today:
#   - fanuc_driver          FANUC's own ros2_control streaming driver (fanuc_libs,
#                           fanuc_hardware_interface, fanuc_moveit_config, ...)
#   - fanuc_description     URDF/meshes for classic FANUC arms (m10, m20, m710, ...)
#   - ros2_fanuc_interface  CRX cobot family (crx10ia/crx20ia/... description +
#                           MoveIt configs), a community fork by paolofrance
#   - seam_ros2_pkg         Crobotic's FANUC/CRX welding-cell bring-up package
#                           (the one the GUI's launch_server actually calls:
#                           crx_gz_sim*, fanuc_moveit_stack, fanuc_hybrid_*)
#
# Build:  docker build -t fanuc_driver:latest .
# Run:    see README.md in this folder

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

# Gazebo + ROS–Gazebo bridges (seam_ros2_pkg's crx_gz_sim*, gz_ros2_control)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends gz-ionic && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-ros-gz-sim \
      ros-${ROS2_DISTRO}-ros-gz-bridge \
      ros-${ROS2_DISTRO}-gz-ros2-control || \
    apt-get install -y --no-install-recommends ros-${ROS2_DISTRO}-ros-gz || true && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/fanuc_ws/src

# Branches match what is currently checked out under /root/arms_ws/src in the
# combined dev container — update here if that container's branches move on.
RUN git clone -b apirsic/devel https://github.com/CroboticSolutions/fanuc_driver.git && \
    git clone -b apirsic/devel https://github.com/CroboticSolutions/fanuc_description.git && \
    git clone -b paolo https://github.com/paolofrance/ros2_fanuc_interface && \
    git clone -b main https://github.com/CroboticSolutions/seam_ros2_pkg.git

WORKDIR /root/fanuc_ws

# seam_ros2_pkg's wrist-camera launch variants (fanuc_hybrid_oak, *_d435, ...)
# exec_depend on depthai_descriptions_v3 / realsense2_camera / realsense2_description
# — rosdep resolves all three to real apt packages (ros-jazzy-depthai-descriptions-v3,
# ros-jazzy-realsense2-camera, ros-jazzy-realsense2-description), so they're pulled
# in automatically here. This is the wrist-mounted camera for the welding cell,
# a different device from the workspace-view OAK camera that runs in its own
# container — no overlap.
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
    echo "source /root/fanuc_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]
