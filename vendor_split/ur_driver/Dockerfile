# ur_driver — Universal Robots ROS 2 driver + Gazebo simulation + MoveIt bring-up.
#
# This is the split-out "everything UR" half of the original combined
# /root/Dockerfile: driver, description, GZ simulation, Robotiq gripper and
# their MoveIt config/move_group/controllers. The arm_api2 container talks to
# whatever this container brings up (move_group, controller_manager) over the
# shared ROS 2 network — see ../README.md.
#
# Build:  docker build -t ur_driver:latest .
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

# Desktop (RViz2, tf, common msgs), MoveIt 2, ros2_control — needed because this
# container also runs ur_moveit_config / move_group for its own namespace(s).
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

# Gazebo + ROS–Gazebo bridges (ur_simulation_gz)
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

WORKDIR /root/ur_ws/src

# Branches match what is currently checked out under /root/arms_ws/src in the
# combined dev container — update here if that container's branches move on.
# NOTE: `serial` currently resolves to RoverRobotics-forks/serial-ros2@master
# in that workspace (NOT tylerjw/serial@ros2, which the original combined
# Dockerfile used) — this file follows the fork actually in use today.
RUN git clone -b multiple_ur_robots https://github.com/CroboticSolutions/Universal_Robots_ROS2_Driver.git && \
    git clone -b multiple_ur_robots https://github.com/CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation.git && \
    git clone -b multiple_ur_robots https://github.com/CroboticSolutions/ros2_robotiq_gripper.git && \
    git clone -b master https://github.com/RoverRobotics-forks/serial-ros2.git serial && \
    git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git

# Jazzy controller_manager spawner treats a bare `robotiq_gripper_controller:` key as an error when
# spawning under /ur1 (FATAL + misleading "not launching"); `/**/robotiq_gripper_controller` is enough.
RUN perl -0777 -i -pe 's/\nrobotiq_gripper_controller:\n  ros__parameters:\n    joint: robotiq_85_left_knuckle_joint\n    state_interfaces:\n      - position\n      - velocity\n    allow_stalling: true\n    stall_timeout: 0.05\n    goal_tolerance: 0.003\n\n/\n/s' \
    /root/ur_ws/src/Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/config/ur_controllers.yaml

WORKDIR /root/ur_ws

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
    echo "source /root/ur_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc

CMD ["bash"]
