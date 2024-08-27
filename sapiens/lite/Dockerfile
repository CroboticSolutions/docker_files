FROM pytorch/pytorch:2.2.0-cuda11.8-cudnn8-devel

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

# Conda
#ENV CONDA_DIR /opt/conda
#RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
#    /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

# Sapiens
RUN git clone https://github.com/facebookresearch/sapiens.git

# Define ENV variables
ENV SAPIENS_ROOT=/root/sapiens
ENV SAPIENS_LITE_ROOT=$SAPIENS_ROOT/lite

# Conda in docker: https://medium.com/@chadlagore/conda-environments-with-docker-82cdc9d25754
# Conda in docker: https://pythonspeed.com/articles/activate-conda-dockerfile/
# Minimal sapiens_lite conda env 
RUN conda init bash \
    && . ~/.bashrc \
    && conda create --name sapiens_lite python=3.10 \
    && conda activate sapiens_lite \
    && conda install torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia \
    && pip install opencv-python tqdm json-tricks mmdet mmcv 

SHELL ["/bin/bash", "-c"]

# Download weights from huggingface

# Sapiens lite installation
RUN cd $SAPIENS

CMD ["bash"]
