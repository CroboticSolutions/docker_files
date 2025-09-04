FROM ubuntu:jammy AS base
LABEL maintainer=filip.zoric@crobotics.tech

# Setup environment
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
#ENV ROS2_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV ROS_DISTRO=noetic
ENV HOME=/root
# TODO: Check how to set ROSCONSOLE_FORMAT in ROS 2
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
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libeigen-stl-containers-dev \
    tar

# TODO: Instead of ROS, install ROS 2
#RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
#RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#RUN apt-get update
#RUN apt install -y ros-$ROS_DISTRO-desktop-full
#RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# To use private ssh repos in Dockerfile: https://stackoverflow.com/a/53548076
#   - build docker with --ssh default
#   - RUN git commands for private repos with --mount=type=ssh
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts

# Bootstrap robotpkg
#  - robotpkg is a compilation framework and packaging system for installing
#    robotics software developed by the robotic community.
WORKDIR $HOME
RUN mkdir -p $HOME/src
WORKDIR $HOME/src
RUN git clone git://git.openrobots.org/robots/robotpkg
WORKDIR $HOME/src/robotpkg/bootstrap
# prefix is pkg installation folder
RUN bash -c "./bootstrap --prefix $HOME/openrobots"
RUN echo "export PATH=$PATH:$HOME/openrobots/bin:$HOME/openrobots/sbin" >> ~/.bashrc

# ROS required packages --> w8 for ros 2
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

# Simulation
RUN apt-get install -y \
    libspdlog-dev \
    libxaw7-dev \
    rapidjson-dev \
    sqlite3 \
    liburdfdom-dev \
    libdart-all-dev \
    qtdeclarative5-dev \
    qml-module-qtgraphicaleffects \
    qml-module-qtquick-controls \
    qtquickcontrols2.5-dev \
    libfreeimage-dev \
    liblz4-dev

# gz-utils requires CMake 3.22.1
WORKDIR $HOME
RUN git clone https://github.com/Kitware/CMake.git
WORKDIR $HOME/CMake
RUN git checkout tags/v3.22.1
RUN ./configure --qt-gui
RUN ./bootstrap \
    && make -j$(nproc) \
    && sudo make install

#WORKDIR $HOME
#RUN apt-get remove -y libspdlog-dev
#RUN git clone https://github.com/gabime/spdlog.git
#WORKDIR $HOME/spdlog
#RUN mkdir build
#RUN git checkout v1.7.0
#WORKDIR $HOME/spdlog/build
#RUN cmake .. -DCMAKE_CXX_FLAGS="-fpic"
#RUN make -j8 && make install

## Also spdlog certain version
#WORKDIR $HOME
#RUN git clone

# genom3 requirements
RUN apt-get install -y \
    python3-nose \
    python3-rosdistro \
    python3-yaml \
    python3-numpy \
    python3-scipy

# Set up sourcing
COPY ./to_copy/env_vars /tmp/env_vars
RUN cat /tmp/env_vars >> ~/.bashrc

# Copy robotconf
#  - configuration file specifying the used packages and their options
COPY ./to_copy/pocolibs_robotpkg.conf $HOME/openrobots/etc/pocolibs_robotpkg.conf
COPY ./to_copy/ros_robotpkg.conf $HOME/openrobots/etc/ros_robotpkg.conf 

# Choose robotpkg conf based on the middleware 
ARG MIDDLEWARE=pocolibs
RUN if [ "$MIDDLEWARE" = "pocolibs" ] ; then \
       mv $HOME/openrobots/etc/pocolibs_robotpkg.conf $HOME/openrobots/etc/robotpkg.conf 
    else \
       mv $HOME/openrobots/etc/ros_robotpkg.conf $HOME/openrobots/etc/robotpkg.conf 
    fi
# If necessary remove confs 
RUN rm $HOME/openrobots/etc/ros_robotpkg.conf && rm $HOME/openrobots/etc/pocolibs_robotpkg.conf

# Build telekyb3
#  - a collection of software packages related to aerial robotics
#  - flight controller, trajectory planner, drivers, etc.
WORKDIR $HOME/src/robotpkg
RUN bash -c "export PATH=$PATH:$HOME/openrobots/bin:$HOME/openrobots/sbin; source ~/.bashrc; make update-telekyb3"

# Install gz-ionic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y gz-ionic

# Install simulation
RUN apt-get install -y \
    libgdal-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
#    libprotoc-dev \
#    protobuf-compiler \
    libczmq-dev \
    libjsoncpp-dev \
    libyaml-dev \
    libzip-dev \
    qml-module-qtquick-controls \
    qml-module-qtquick-dialogs \
    liblz4-dev \
    doxygen \
    graphviz

# Build simulation
RUN bash -c "export PATH=$PATH:$HOME/openrobots/bin:$HOME/openrobots/sbin; source ~/.bashrc; make update-simulation"

# Build genom3
#  - a software component generator that can target different middlewares (such
#    as ROS, YARP) so that the components source code remains independant of the
#    middleware.
RUN bash -c "export PATH=$PATH:$HOME/openrobots/bin:$HOME/openrobots/sbin; source ~/.bashrc; make update-genom3"

# Copy helper scripts and environment variables
WORKDIR $HOME
RUN mkdir scripts
WORKDIR $HOME/scripts
COPY ./scripts/tk3-quadrotor-simulation.sh .
COPY ./scripts/quadrotor_ctl.py .
COPY ./scripts/env.sh .
COPY ./scripts/genom_ndt.yml /root/.config/tmuxinator/ 
COPY ./scripts/ros_genom_ndt.yml /root/.config/tmuxinator/
RUN echo "source $HOME/scripts/env.sh" >> ~/.bashrc

# Create devel and clone UT air-phynt-genom3
#  - air-phynt-genom3 is a genom3 component for aerial physical interaction
WORKDIR $HOME
RUN mkdir devel
WORKDIR $HOME/devel
RUN apt-get -y install asciidoctor
RUN git clone https://gitlab.com/AIR-TEAM/air-phynt-genom3.git
RUN cd air-phynt-genom3 && git checkout experimental

# Build air-phynt-genom3
ENV DEVEL_BASE=$HOME/devel
WORKDIR $HOME/devel/air-phynt-genom3
RUN bash -c "./bootstrap.sh"

RUN mkdir build
WORKDIR $HOME/devel/air-phynt-genom3/build

ARG MIDDLEWARE=pocolibs
RUN if [ "$MIDDLEWARE" = "pocolibs" ] ; then \
       bash -c ". $HOME/scripts/env.sh && ../configure --prefix=$DEVEL_BASE --with-templates=pocolibs/client/c,pocolibs/server && make install"; \
    else \
       bash -c "source ~/.bashrc && ../configure --prefix=$DEVEL_BASE --with-templates=ros/client/c,ros/server,ros/client/ros && make install"; \
    fi

# Clone minithex-cntrl-python-scripts
#  - a repository containing Python scripts for controlling the minithex robot
WORKDIR $HOME/devel/
RUN --mount=type=ssh git clone git@gitlab.com:AIR-TEAM/minithex-cntrl-python-scripts.git
WORKDIR $HOME/devel/minithex-cntrl-python-scripts/
RUN git checkout autoassess_gz_new

RUN mkdir -p $HOME/logs
WORKDIR $HOME/scripts
CMD ["bash"]
