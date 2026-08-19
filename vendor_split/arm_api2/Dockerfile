# arm_api2 — vendor-agnostic MoveIt interface node.
#
# Contains ONLY arm_api2 + arm_api2_msgs. No vendor driver, no Gazebo, no
# simulation stack — this container's launch_iface node talks to whichever
# driver container (ur_driver / piper_driver / fanuc_driver / abb_driver) is
# running move_group + controllers for a given robot namespace, over ROS 2
# DDS on the shared host network. See ../README.md for the networking
# contract shared by all 5 containers.
#
# Build:  docker build -t arm_api2:latest .
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

# ros_base + MoveIt (MoveGroupInterface client libs) + Cyclone DDS. No ros2_control,
# no Gazebo, no RViz/desktop GUI — those belong to whichever driver container
# actually runs the controllers/simulation.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS2_DISTRO}-ros-base \
      ros-${ROS2_DISTRO}-moveit \
      ros-${ROS2_DISTRO}-moveit-visual-tools \
      ros-${ROS2_DISTRO}-moveit-servo \
      ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS2_DISTRO}-tf-transformations \
      python3-colcon-common-extensions \
      python3-vcstool \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/arm_api2_ws/src

# Branches match what is currently checked out under /root/arms_ws/src in the
# combined dev container — update here if that container's branches move on.
RUN git clone -b apirsic/devel https://github.com/CroboticSolutions/arm_api2.git && \
    git clone -b apirsic/devel https://github.com/CroboticSolutions/arm_api2_msgs.git

WORKDIR /root/arm_api2_ws

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
    echo "source /root/arm_api2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]
