FROM ubuntu:jammy AS ros2

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
# Not sure this is same in ROS and ROS 2 
# ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    bison \
    bzip2 \
    cmake \
    curl \
    flex \
    git \
    gnupg \
    lsb-release \
    libbz2-dev \
    libelf-dev \
    libudev-dev \
    libusb-1.0.0-dev \
    libboost-dev \
    libncurses-dev \
    libsdl2-dev \
    libtool-bin \
    pax \
    sudo \
    nano \
    net-tools \
    tmux \
    tmuxinator \
    tcl-dev \
    tk-dev \
    wget \
    ranger \
    htop \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libeigen-stl-containers-dev \
    tar 

# Prepare for ROS2
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
#RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
#RUN apt update

# Install ROS2
# RUN apt-get update && \
#    apt install -y \
#    ros-${ROS2_DISTRO}-desktop-full \
#    ros-${ROS2_DISTRO}-test-msgs \
#    ros-${ROS2_DISTRO}-generate-parameter-library 

# Install ROS2 tools
#RUN apt install -y \
#    python3-argcomplete \
#    ros-dev-tools \
#    python3-colcon-common-extensions \
#    python3-colcon-mixin \
#    python3-vcstool 

# Install middleware 
#RUN sudo apt install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
#RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Gazebo fortress installation
RUN apt-get update
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get -y update
RUN apt-get install -y gz-harmonic

ENV HOME=/root

# Bootstrap robotpkg
WORKDIR /root/
RUN mkdir -p /sw/srcarchiveers
WORKDIR /root/sw/src
RUN git clone https://git.openrobots.org/robots/robotpkg
WORKDIR /root/sw/src/robotpkg/bootstrap
RUN ls
RUN bash -c "./bootstrap --prefix /root/openrobots"
RUN echo "export PATH=$PATH:/root/openrobots/bin:/root/openrobots/sbin" >> ~/.bashrc

# Copy needed robotpkg.conf to the default conf file
COPY ./to_copy/robotpkg.conf /root/openrobots/etc/robotpkg.conf

WORKDIR /root/sw/src/robotpkg

# ROS required packages --> w8 for ros 2
#RUN apt-get install -y \
#    python3.10 \
#    python3-dev \
#    python3-pyparsing \
#    python3-setuptools \
#    libgtest-dev \
#    python3-empy

# For some reason this command fails --> Fails in docker but not in container?!
#RUN bash -c "source ~/.bashrc; make update-telekyb3"

CMD ["bash"]
