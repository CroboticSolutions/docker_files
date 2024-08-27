FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

# Setup environment
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb
ENV ROSCONSOLE_FORMAT '[${severity}] [${time}] [${node}]: ${message}'

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
    build-essential \
    wget \
    git \
    feh \
    ffmpeg \
    libsm6 \
    libxext6 

WORKDIR /root

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

ENV PATH=$CONDA_DIR/bin:$PATH

# Sapiens
RUN git clone https://github.com/fzoric8/sapiens.git

# Define ENV variables
ENV SAPIENS_ROOT=/root/sapiens
ENV SAPIENS_LITE_ROOT=$SAPIENS_ROOT/lite

# Fixlibmamba stuff 
RUN conda install --solver=classic conda-forge::conda-libmamba-solver conda-forge::libmamba conda-forge::libmambapy conda-forge::libarchive
WORKDIR /root/sapiens/_install
RUN bash -c "./conda.sh"

# Sapiens lite installation
RUN cd $SAPIENS

CMD ["bash"]
