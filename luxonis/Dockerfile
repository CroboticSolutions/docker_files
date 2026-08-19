FROM ubuntu:jammy AS ros2_base

# Builds depthai-ros (humble branch, v2.12.2) for an OAK-D SR camera.
# See README.md in this folder for build/run instructions and topic reference.

# Setup environment
ENV HOME=/root

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS2_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Zagreb
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages
RUN apt-get update && apt-get install -q -y \
    apt-utils \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    sudo \
    nano \
    net-tools \
    tmux \
    wget \
    htop \
    libusb-1.0-0-dev \
    python3-pip

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update

# Install ROS2
RUN apt-get install -y \
    ros-${ROS2_DISTRO}-desktop-full

# Install ROS2 tools
RUN apt-get install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool \
    python3-rosdep

# Colcon setup
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Install middleware. Must match the other host-networked ROS2 containers on this
# machine (e.g. composers/demomotion) so DDS discovery finds this container's topics.
RUN apt-get install -y ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# ffmpeg_image_transport_msgs is a depthai_ros_driver dependency; it's occasionally
# missing from the apt mirror, in which case fall back to building it from source.
RUN apt-get update && \
    (apt-get install -y ros-${ROS2_DISTRO}-ffmpeg-image-transport-msgs || true)

RUN rosdep init || true
RUN rosdep update

ENV DEPTHAI_WS=/root/depthai_ws
RUN mkdir -p ${DEPTHAI_WS}/src
WORKDIR ${DEPTHAI_WS}/src

RUN git clone --branch humble https://github.com/luxonis/depthai-ros.git depthai-ros

# Fallback if the apt package above was unavailable
RUN dpkg -s ros-${ROS2_DISTRO}-ffmpeg-image-transport-msgs > /dev/null 2>&1 || \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs.git ffmpeg_image_transport_msgs

# depthai_examples breaks the build (conflicting deps) and isn't needed to run the
# driver, so drop the dependency on it before resolving/building the workspace.
RUN sed -i '/<depend>depthai_examples<\/depend>/d' ${DEPTHAI_WS}/src/depthai-ros/depthai_ros_driver/package.xml && \
    sed -i '/<exec_depend>depthai_examples<\/exec_depend>/d' ${DEPTHAI_WS}/src/depthai-ros/depthai-ros/package.xml

# OAK-D SR has no RGB sensor: force the Depth pipeline and publish the raw +
# rectified right mono image so it lines up with /oak/stereo/image_raw (both are
# aligned to CAM_C / the right camera on this variant).
COPY to_copy/oak_d_sr.yaml ${DEPTHAI_WS}/src/depthai-ros/depthai_ros_driver/config/oak_d_sr.yaml

WORKDIR ${DEPTHAI_WS}
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys depthai_examples

# colcon's cmake step needs empy's old (3.x) API under Humble; empy 4.x breaks it.
RUN pip3 install catkin_pkg empy==3.3.4 lark

RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    MAKEFLAGS="-j2" colcon build --symlink-install \
      --parallel-workers 1 \
      --packages-skip depthai_examples \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "" >> ~/.bashrc && \
    echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ${DEPTHAI_WS}/install/setup.bash" >> ~/.bashrc

WORKDIR ${DEPTHAI_WS}
CMD ["bash"]
