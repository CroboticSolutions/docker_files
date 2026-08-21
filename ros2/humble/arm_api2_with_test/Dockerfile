# arm_api2 selftest image (ROS 2 Humble, Ubuntu jammy).
#
# Build (no Gazebo):  docker build -t arm_api2:selftest .
# Build (with Gazebo): docker build -t arm_api2:selftest --build-arg ENABLE_GAZEBO=1 .
# Run:    ./first-run.sh   (expects image arm_api2:selftest, container arm_api2_selftest — see README)
#
# Selftest: https://github.com/CroboticSolutions/arm_api2/blob/humble/tests/arm_api2_selftest/README.md

FROM ubuntu:jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG ROS2_DISTRO=humble
ARG ENABLE_GAZEBO=0

ENV HOME=/root \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS2_DISTRO=${ROS2_DISTRO} \
    ROS_DISTRO=${ROS2_DISTRO} \
    ENABLE_GAZEBO=${ENABLE_GAZEBO} \
    DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Zagreb \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Base tools + GL stacks used by RViz (and Gazebo when enabled)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo "${TZ}" > /etc/timezone && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
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

# Desktop (RViz2, tf, common msgs), MoveIt 2, ros2_control
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

# Optional Gazebo Fortress. ROS–Gazebo apt packages come from rosdep once
# Universal_Robots_ROS2_GZ_Simulation is cloned below.
RUN if [ "${ENABLE_GAZEBO}" = "1" ]; then \
      curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
      apt-get update && \
      apt-get install -y --no-install-recommends ignition-fortress && \
      rm -rf /var/lib/apt/lists/*; \
    fi

# Clone workspace sources at the branches actually used (latest of each branch, not a SHA pin).
# ros2_robotiq_gripper must stay on PickNik humble: Crobotic multiple_ur_robots
# targets Jazzy ros2_control (get_optional / set_value bool) and will not compile.
WORKDIR /root/arms_ws/src
RUN git clone --depth 1 --single-branch --branch humble \
      https://github.com/CroboticSolutions/arm_api2.git && \
    git clone --depth 1 --single-branch --branch apirsic/devel \
      https://github.com/CroboticSolutions/arm_api2_msgs.git && \
    git clone --depth 1 --single-branch --branch humble \
      https://github.com/CroboticSolutions/Universal_Robots_ROS2_Driver.git && \
    git clone --depth 1 --single-branch --branch humble \
      https://github.com/PickNikRobotics/ros2_robotiq_gripper.git && \
    git clone --depth 1 --single-branch --branch ros2 \
      https://github.com/tylerjw/serial.git && \
    git clone --depth 1 --single-branch --branch humble \
      https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git && \
    if [ "${ENABLE_GAZEBO}" = "1" ]; then \
      git clone --depth 1 --single-branch --branch humble \
        https://github.com/CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation.git; \
    fi

WORKDIR /root/arms_ws

# arm_api2 humble includes yaml-cpp via MoveIt headers but never links the
# library. Apt MoveIt does not export that dependency, so the linker fails.
RUN python3 - <<'PY'
from pathlib import Path

cmake = Path("/root/arms_ws/src/arm_api2/CMakeLists.txt")
text = cmake.read_text()
marker = "target_link_libraries(moveit2_iface yaml-cpp)"
if marker not in text:
    needle = "ament_package()"
    if needle not in text:
        raise SystemExit("ament_package() not found in arm_api2 CMakeLists.txt")
    patch = (
        "find_package(yaml-cpp REQUIRED)\n"
        f"{marker}\n\n"
    )
    cmake.write_text(text.replace(needle, patch + needle, 1))
PY

# Earlier layers run `rm -rf /var/lib/apt/lists/*`; rosdep invokes apt without an update.
# Refresh indexes first or every rosdep key fails with "Unable to locate package".
RUN apt-get update && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install -y --no-install-recommends libyaml-cpp-dev && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
      empy \
      catkin_pkg \
      lark

RUN source /opt/ros/${ROS2_DISTRO}/setup.bash && \
    colcon build --symlink-install

RUN echo "" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/arms_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >> /root/.bashrc && \
    if [ "${ENABLE_GAZEBO}" = "1" ]; then \
      echo "export GZ_SIM_RESOURCE_PATH=/root/arms_ws/install/robotiq_description/share" >> /root/.bashrc; \
    fi

CMD ["bash"]
