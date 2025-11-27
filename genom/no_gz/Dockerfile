FROM ubuntu:jammy

# Setup environment
ENV LANG=C.UTF-8
ENV LC_AL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV ROS_DISTRO=noetic
ENV HOME=/root
# Not sure this is same in ROS and ROS 2 
# ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

# Mitigate interactive prompt for choosing keyboard type
COPY ./to_copy/keyboard /etc/default/keyboard

# Setup timezone (fix interactive package installation)
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

# Install necessary packages for ROS (no Gazebo)
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
    libboost-all-dev \
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
    lsb-release \
    libeigen-stl-containers-dev \
    tar \
    mlocate 

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# To use private ssh repos in Dockerfile: https://stackoverflow.com/a/53548076
# - build docker with --ssh default
# - RUN git commands for private repos with --mount=type=ssh
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts

# Bootstrap robotpkg
WORKDIR $HOME
RUN mkdir -p $HOME/src
WORKDIR $HOME/src
RUN git clone git://git.openrobots.org/robots/robotpkg
WORKDIR $HOME/src/robotpkg/bootstrap
RUN bash -c "./bootstrap --prefix /root/openrobots"
ENV PATH="/root/openrobots/bin:/root/openrobots/sbin:${PATH}"

RUN apt-get install -y \
    #python3.10 \
    python3-dev \
    python3-pyparsing \
    python3-setuptools \
    libgtest-dev \
    python3-empy \
    libconsole-bridge-dev \
    libpoco-dev \
    libapr1-dev \
    libgpgme-dev \
    liblog4cxx-dev \
    python3-pycryptodome \
    python3-defusedxml \
    python3-gnupg \
    python3-netifaces \
    python3-paramiko

# genom3 requirements
RUN apt-get install -y \
    python3-nose \
    python3-rosdistro \
    python3-yaml \
    python3-numpy \
    python3-scipy

# Copy robotpkg configuration for ROS middleware
COPY ./to_copy/ros_robotpkg.conf $HOME/openrobots/etc/robotpkg.conf

# run libtinyxml2
RUN apt-get install -y \
    libtinyxml2-dev \
    liblz4-dev

# robotpkg installation [teleykb3 installation]
WORKDIR $HOME/src/robotpkg
RUN make update-telekyb3

# Build genom3
WORKDIR $HOME/src/robotpkg
RUN make update-genom3

# Source env variable setup
RUN mkdir -p $HOME/scripts
COPY ./scripts/env.sh /root/scripts/env.sh
RUN cat /root/scripts/env.sh >> ~/.bashrc

# Build phynt for devel
WORKDIR $HOME/devel/
RUN --mount=type=ssh git clone git@gitlab.com:AIR-TEAM/air-phynt-genom3.git

# Add phynt build 
WORKDIR $HOME/devel/air-phynt-genom3
RUN bash -c "./bootstrap.sh"
RUN apt-get install -y asciidoctor

RUN mkdir build
WORKDIR $HOME/devel/air-phynt-genom3/build
RUN bash -c 'set -e; \
    . "$HOME/scripts/env.sh" && \
    ../configure --prefix="$DEVEL_BASE" --with-templates=ros/client/c,ros/server,ros/client/ros && \
    make install'

# Clone minithex-cntrl-python-scripts
WORKDIR $HOME/devel 
RUN --mount=type=ssh git clone git@gitlab.com:AIR-TEAM/minithex-cntrl-python-scripts.git
WORKDIR $HOME/devel/minithex-cntrl-python-scripts/
RUN git checkout autoassess_gz_classic 
WORKDIR $HOME/devel

# Install pip and matplotlib
RUN apt-get install -y python3-pip
RUN pip install 'matplotlib>=3.5,<3.8' minsnap_trajectories

# Setup genom_ros_bridge (creates catkin_ws with genom_ros_ctl, ros_gui_bridge, rospy_message_converter)
COPY ./scripts/setup_genom_ros_bridge.sh $HOME/scripts/setup_genom_ros_bridge.sh
RUN chmod +x $HOME/scripts/setup_genom_ros_bridge.sh
ARG SETUP_ROS_GENOM_BRIDGE=true
RUN --mount=type=ssh bash -c 'if [ "$SETUP_ROS_GENOM_BRIDGE" = "true" ]; then \
    source /root/openrobots/etc/ros/setup.bash && \
    bash $HOME/scripts/setup_genom_ros_bridge.sh; \
  else \
    echo "Skipping genom_ros_bridge setup"; \
  fi'

CMD ["bash"]
