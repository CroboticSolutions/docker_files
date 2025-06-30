FROM nvidia/cuda:12.4.0-runtime-ubuntu22.04

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
    cmake \
    curl \
    cuda-toolkit \
    git \
    gnupg \
    lsb-release \
    libboost-dev \
    sudo \
    nano \
    net-tools \
    tmux \
    tmuxinator \
    wget \
    ranger \
    htop \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libeigen-stl-containers-dev 

# Prepare for ROS2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

# Fix sources bug
RUN rm /etc/apt/sources.list.d/cuda-ubuntu2204-x86_64.list

# Install ROS2
RUN apt-get update && \
    apt install -y \
    ros-${ROS2_DISTRO}-desktop-full \
    ros-${ROS2_DISTRO}-test-msgs \
    ros-${ROS2_DISTRO}-generate-parameter-library 

# Install ROS2 tools
RUN apt install -y \
    python3-argcomplete \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-vcstool 

# Colcon setup according to moveit tutorial 
RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
RUN colcon mixin update default

# Copy configuration files
COPY ./to_copy/aliases /root/.bash_aliases
COPY ./to_copy/nanorc /root/.nanorc
COPY ./to_copy/tmux /root/.tmux.conf
COPY ./to_copy/ranger /root/.config/ranger/rc.conf

# Gazebo fortress installation
RUN apt-get update
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install -y \ 
    ignition-fortress 

RUN apt-get install -y \
    ros-${ROS2_DISTRO}-ackermann-msgs \
    ros-${ROS2_DISTRO}-backward-ros

# Build ROS2 control --> Built inside arms_ws
# RUN mkdir -p /root/ros2_ws/src
# WORKDIR /root/ros2_ws
# RUN vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.humble.repos src
# RUN sudo rosdep init && rosdep update --rosdistro=${ROS2_DISTRO}
# RUN apt-get update
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN bash -c ". /opt/ros/${ROS2_DISTRO}/setup.sh; colcon build --symlink-install"

# Modify .bashrc
RUN echo "" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc
#RUN echo "source /root/ros2_ws/install/local_setup.bash" >> ~/.bashrc

# MoveIt 2 ws
WORKDIR /root
ENV COLCON_WS /root/ws_moveit2
RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}/src

# Download moveit2 source code
RUN git clone https://github.com/ros-planning/moveit2.git -b ${ROS2_DISTRO}
RUN git clone https://github.com/moveit/py_binding_tools.git -b ros2
RUN for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_${ROS2_DISTRO}.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
RUN sudo rosdep init 
RUN rosdep update
RUN rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS2_DISTRO} -y
#RUN git clone https://github.com/moveit/moveit_task_constructor.git -b ${ROS2_DISTRO}

# Install middleware 
RUN sudo apt install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
RUN export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Build MoveIt!
RUN sudo apt install -y \
    ros-${ROS2_DISTRO}-ament-cmake-python \
    ros-${ROS2_DISTRO}-rviz-visual-tools \
    ros-${ROS2_DISTRO}-vision-msgs 

WORKDIR ${COLCON_WS}/src
RUN bash -c "source /opt/ros/humble/setup.sh; source ~/.bashrc; MAKEFLAGS="-j${nproc-1}" colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN git clone https://github.com/moveit/moveit2_tutorials.git -b ${ROS2_DISTRO}
RUN git clone https://github.com/moveit/moveit_visual_tools.git -b ros2
RUN git clone https://github.com/PickNikRobotics/graph_msgs.git -b ros2
RUN git clone https://github.com/moveit/moveit_task_constructor.git -b ${ROS2_DISTRO}
#RUN git clone https://github.com/moveit/moveit_resources.git -b ros2
WORKDIR ${COLCON_WS}
RUN bash -c "source /opt/ros/humble/setup.sh; source ~/.bashrc; MAKEFLAGS="-j${nproc-1}" colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release"
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

# Install UR, Kinova, Robotiq gripper 
RUN apt-get install -y \
    libserial-dev \
    ament-cmake
RUN mkdir -p arm_ws/src
WORKDIR /root/arm_ws/src
RUN git clone --single-branch --branch ros2 https://github.com/tylerjw/serial
WORKDIR /root/arm_ws/src/serial
RUN make && make install

# Clone repositories
WORKDIR /root/arm_ws/src 
RUN git clone --single-branch --branch humble https://github.com/PickNikRobotics/ros2_robotiq_gripper.git
RUN git clone --single-branch --branch humble https://github.com/CroboticSolutions/ros2_kortex.git 
RUN git clone --single-branch --branch humble https://github.com/CroboticSolutions/Universal_Robots_ROS2_Driver.git
RUN git clone --single-branch --branch humble https://github.com/CroboticSolutions/Universal_Robots_ROS2_GZ_Simulation.git

# Update deps 
WORKDIR /root/arm_ws
ENV ROS_DISTRO humble
RUN rosdep update && rosdep install --ignore-src --from-paths src -y

# Only panda repo (to mitigate dep install which install moveit from pkgs)
WORKDIR /root/arm_ws/src
RUN git clone --single-branch --branch humble_devel https://github.com/AndrejOrsula/panda_gz_moveit2

# Build arm_ws
WORKDIR /root/arm_ws/
RUN bash -c "source /opt/ros/humble/setup.bash; rosdep update && rosdep install --ignore-src --from-paths src -y"
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"
RUN echo "source /root/arm_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=/root/arm_ws/install/robotiq_description/share" >> ~/.bashrc
COPY ./to_copy/ros2_aliases /tmp/
RUN cat /tmp/ros2_aliases >> ~/.bashrc 

CMD ["bash"]

# Cumotion
WORKDIR /root
RUN mkdir -p isaac_ros_ws/src
ENV ISAAC_ROS_WS=/root/isaac_ros_ws
WORKDIR ${ISAAC_ROS_WS}/src
RUN apt install gnupg
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
RUN apt install -y software-properties-common 
RUN add-apt-repository 'deb https://repo.download.nvidia.com/jetson/x86_64/jammy r36.4 main'
RUN apt update
RUN apt install libnvvpi3 vpi3-dev vpi3-samples
RUN git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
ENV CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-12.4
RUN bash -c "source /opt/ros/humble/setup.bash; colcon build" 
RUN git clone --recurse-submodules -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git isaac_ros_cumotion
RUN apt-get update
RUN rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_common --ignore-src -y
# This is just bad. Try their included Dockerfile, otherwise, rest :)
#RUN rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_cumotion --ignore-src -y
#RUN apt-get install -y python3-pip
#RUN pip3 install torch
#RUN bash -c "source /opt/ros/humble/setup.bash; colcon build --packages-up-to isaac_ros_cumotion_examples" 

CMD ["bash"]
