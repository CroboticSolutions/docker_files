 FROM ubuntu:noble AS ros2_base

 # Setup environment
 ENV HOME=/root

 ENV LANG=C.UTF-8
 ENV LC_ALL=C.UTF-8
 ENV ROS2_DISTRO=jazzy
 ENV DEBIAN_FRONTEND=noninteractive
 ENV NVIDIA_VISIBLE_DEVICES=all
 ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
 ENV TZ=Europe/Zagreb
 ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
 ARG PY_VENV=${HOME}/py_global

 RUN --mount=type=ssh

 # Setup timezone (fix interactive package installation)
 RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone

 # Install necessary packages for ROS
 RUN apt-get update &&  apt-get install -q -y \
     apt-utils \
     build-essential \
     bc \
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
     libgl1 \
     libegl1 \
     libglu1-mesa \
     mesa-utils \
     x11-apps \
     python3-venv \
     python3-pip \
     python3-tk

 # Set up Python virtual environment
 RUN python3 -m venv ${PY_VENV} --system-site-packages
 ENV PATH="${PY_VENV}/bin:$PATH"

 # Prepare for ROS2
 RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
 RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
 RUN apt-get update

 # Install ROS2
 RUN apt-get update && apt-get install -y \
     ros-${ROS2_DISTRO}-desktop-full

 # Install ROS2 tools
 RUN apt-get install -y \
     python3-argcomplete \
     ros-dev-tools \
     python3-colcon-common-extensions \
     python3-colcon-mixin \
     python3-vcstool

 # Colcon setup
 RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
 RUN colcon mixin update default

 # Install camera and visualization packages
 RUN apt-get install -y \
     ros-${ROS2_DISTRO}-usb-cam \
     ros-${ROS2_DISTRO}-image-transport \
     ros-${ROS2_DISTRO}-image-transport-plugins \
     ros-${ROS2_DISTRO}-cv-bridge \
     ros-${ROS2_DISTRO}-rqt-image-view \
     ros-${ROS2_DISTRO}-visualization-msgs \
     python3-opencv \
     libopencv-dev

 # Modify .bashrc
 RUN echo "" >> ~/.bashrc
 RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> ~/.bashrc

 # Install middleware
 RUN apt-get install ros-${ROS2_DISTRO}-rmw-cyclonedds-cpp -y
 ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
 RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

 # Install MediaPipe and related Python packages with compatible versions
 # mediapipe 0.10.14: Has framework.formats.landmark_pb2 module required by mp_ros_wrapper
 # numpy 1.26.4: Compatible with system matplotlib (compiled with numpy 1.x)
 # opencv-python 4.8.1.78: Compatible with numpy 1.26.4
 RUN pip3 install --no-cache-dir \
     mediapipe==0.10.14 \
     numpy==1.26.4 \
     opencv-python==4.8.1.78

 # Clone hpe_ws packages
 WORKDIR /root/hpe_ws/src
 RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
 RUN --mount=type=ssh git clone -b apirsic/devel git@github.com:CroboticSolutions/mp_ros2_wrapper.git mp_ros_wrapper
 RUN git clone -b apirsic/devel https://github.com/CroboticSolutions/hpe_ros_msgs.git
 RUN --mount=type=ssh git clone -b apirsic/devel git@github.com:CroboticSolutions/usb_cam.git

 ENV ROS_DISTRO=jazzy
 # rosdep update
 WORKDIR /root/hpe_ws
 RUN rosdep init || true
 RUN rosdep update && \
     rosdep install --from-paths src --ignore-src -r -y

 # Install necessary Python packages
 RUN pip3 install \
     empy \
     catkin_pkg \
     lark

 # Build workspace
 RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
     python3 -m colcon build --symlink-install"

 # Add hpe_ws workspace to bashrc
 RUN echo "" >> ~/.bashrc && \
     echo "# HPE workspace" >> ~/.bashrc && \
     echo "source $HOME/hpe_ws/install/setup.bash" >> ~/.bashrc

 RUN echo "source ${PY_VENV}/bin/activate" >> $HOME/.bashrc

 WORKDIR /root/hpe_ws

 CMD ["bash"]
