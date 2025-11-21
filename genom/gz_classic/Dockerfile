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

# Install gazebo classic 
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y \
    gazebo \
    libgazebo-dev 

# Set up sourcing
COPY ./to_copy/env_vars /tmp/env_vars
RUN cat /tmp/env_vars >> ~/.bashrc

# Copy robotconf
#  - configuration file specifying the used packages and their options
RUN echo "Repeat this to refresh also mrsim-gazebo"
COPY ./to_copy/pocolibs_robotpkg.conf $HOME/openrobots/etc/pocolibs_robotpkg.conf
COPY ./to_copy/ros_robotpkg.conf $HOME/openrobots/etc/ros_robotpkg.conf 

# Choose robotpkg conf based on the middleware 
ARG MIDDLEWARE=ros
RUN bash -c 'if [ "$MIDDLEWARE" = "pocolibs" ]; then \
      mv "$HOME/openrobots/etc/pocolibs_robotpkg.conf" "$HOME/openrobots/etc/robotpkg.conf"; \
    else \
      mv "$HOME/openrobots/etc/ros_robotpkg.conf" "$HOME/openrobots/etc/robotpkg.conf"; \
    fi'

# run libtinyxml2
RUN apt-get install -y \
    libtinyxml2-dev \
    liblz4-dev

# robotpkg installation [teleykb3 installation]
WORKDIR $HOME/src/robotpkg
RUN make update-telekyb3

# Install simulation
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
    libfreeimage-dev 

RUN apt-get install -y \
    libgdal-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libprotoc-dev \
    protobuf-compiler \
    libczmq-dev \
    libjsoncpp-dev \
    libyaml-dev \
    libzip-dev \
    qml-module-qtquick-controls \
    qml-module-qtquick-dialogs \
    doxygen \
    graphviz \
    asciidoctor

# Build simulation and genom3
WORKDIR $HOME/src/robotpkg
RUN make update-simulation
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

RUN mkdir build
WORKDIR $HOME/devel/air-phynt-genom3/build
RUN bash -c 'set -e; \
  if [ "$MIDDLEWARE" = "pocolibs" ]; then \
    . "$HOME/scripts/env.sh" && \
    ../configure --prefix="$DEVEL_BASE" --with-templates=pocolibs/client/c,pocolibs/server && \
    make install; \
  else \
    . "$HOME/scripts/env.sh" && \
    ../configure --prefix="$DEVEL_BASE" --with-templates=ros/client/c,ros/server,ros/client/ros && \
    make install; \
  fi'

# Clone minithex-cntrl-python-scripts
WORKDIR $HOME/devel 
RUN --mount=type=ssh git clone git@gitlab.com:AIR-TEAM/minithex-cntrl-python-scripts.git

# Install pip and matplotlib
RUN apt-get install -y python3-pip
RUN pip install 'matplotlib>=3.5,<3.8'

# Clone minsnap trajectories
#WORKDIR $HOME/devel/
#RUN --mount=type=ssh git clone git@gitlab.com:AIR-TEAM/minsnap-trajectories.git

# TODO: 
# [x] Start gazebo simulation
# [ ] Add environment variables sourcing
# [ ] Add tmuxinator script to run it one go 
# [ ] Sort gazebo plugin registration  
# [ ] Add installation of the asciidoctor and graphviz for docs building
# [ ] Add installation of the phynt

CMD ["bash"]
