FROM ubuntu:jammy

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV TZ=Europe/Zagreb

# Install necessary packages for ROS and Gazebo
RUN apt-get update &&  apt-get install -q -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
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
    htop 

# Install miniconda
ENV CONDA_DIR=/opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda init bash \
   && . ~/.bashrc \
   && conda create -n ditto python=3.8 -y \
   && conda activate ditto \
   && pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117

CMD ["bash"]
