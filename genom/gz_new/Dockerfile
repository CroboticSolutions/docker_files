FROM ubuntu:jammy AS base
LABEL maintainer filip.zoric@crobotics.tech

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV ROS2_DISTRO humble
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
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

# Bootstrap robotpkg
WORKDIR $HOME
RUN mkdir -p $HOME/src
WORKDIR $HOME/src
RUN git clone git://git.openrobots.org/robots/robotpkg
WORKDIR $HOME/src/robotpkg/bootstrap
# prefix is pkg installation folder
RUN bash -c "./bootstrap --prefix /root/openrobots"
RUN echo "export PATH=$PATH:/root/openrobots/bin:/root/openrobots/sbin" >> ~/.bashrc

# Copy needed robotpkg.conf to the default conf file

WORKDIR /root/src/robotpkg

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
RUN apt-get update 
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

# genom3 
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
COPY ./to_copy/robotpkg.conf /root/openrobots/etc/robotpkg.conf

# For some reason this command fails --> Fails in docker but not in container?!
WORKDIR /root/src/robotpkg
# Build telekyb
RUN bash -c "export PATH=$PATH:/root/openrobots/bin:/root/openrobots/sbin; source ~/.bashrc; make update-telekyb3"


# Install gz-ionic
RUN echo "test"
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
    liblz4-dev
    
# Build simulation 
RUN bash -c "export PATH=$PATH:/root/openrobots/bin:/root/openrobots/sbin; source ~/.bashrc; make update-simulation"

# Build genom3 
RUN bash -c "export PATH=$PATH:/root/openrobots/bin:/root/openrobots/sbin; source ~/.bashrc; make update-genom3"

# Copy helper scripts
WORKDIR /root
RUN mkdir  scripts 
WORKDIR /root/scripts
COPY ./scripts/tk3-quadrotor-simulation.sh .
COPY ./scripts/quadrotor_ctl.py .
COPY ./scripts/env.sh .

# Create devel and clone UT air-phynt-genom 
WORKDIR /root 
RUN mkdir devel
WORKDIR /root/devel
RUN apt-get -y install asciidoctor
RUN git clone https://gitlab.com/AIR-TEAM/air-phynt-genom3.git 
# Build air-phynt-genom3 
RUN echo "export DEVEL_BASE=/root/devel" >> ~/.bashrc
WORKDIR /root/devel/air-phynt-genom3
RUN git checkout experimental
RUN bash -c "./bootstrap.sh"
RUN mkdir build
WORKDIR /root/devel/air-phynt-genom3/build
# BUILD OF THE CUSTOM GENOM PACKAGE IN DOCKER FAILS (WRONG ENV VARS!)
# POCOLIBS BUILD
# RUN bash -c "./root/scripts/env.sh; ../configure --prefix=$DEVEL_BASE --with-templates=pocolibs/client/c,pocolibs/server"
# ROS BUILD
# RUN bash -c "source ~/.bashrc; ../configure --prefix=$DEVEL_BASE --with-templates=ros/client/c,ros/server,ros/client/ros"
#RUN make install 

WORKDIR /root/devel/
RUN git clone https://gitlab.com/AIR-TEAM/minithex-cntrl-python-scripts.git
WORKDIR /root/devel/minithex-cntrl-python-scripts/
RUN git checkout autoassess_gz_new 

CMD ["bash"]
