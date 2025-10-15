# docker_files 

Repository for useful virtualization (docker) stuff. 
Aimed to make development: 
- easy
- simple
- reproducible

## How to use repository? 

First download and install Docker. Linux is recommended. 

Follow instructions using [apt-repository](https://docs.docker.com/engine/install/ubuntu/). 

After docker installation, follow [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/). 

## How to use hardware acceleration (GPU) with docker? 

### NVIDIA
Follow instructions for installing `nvidia-container-toolkit` via (/apt)[https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html]. 
### AMD
Simnilar to NVIDIA, AMD also provides `container-toolkit`, install it and use it following this (instructions)[https://instinct.docs.amd.com/projects/container-toolkit/en/latest/container-runtime/migration-guide.html]. 

## Basic docker usage/steps?

1. Build `Dockerfile` into docker image using `docker build` command.
2. Use docker image to run docker container (use `first_run.sh` script for recommended init configuration) 
3. Use docker container for development (use `docker start` for starting container, and `docker exec` to enter running container)
4. After done with development use `docker stop` to stop docker container

### Building docker image

```
docker build -t <img_name>:<tag> 
```

### Running docker container (first time) 
Each `Dockerfile` has corresponding `first_run.sh` script that can be used. 
Each `first_run.sh` contains following command: 
```
#!/bin/bash

CONTAINER_NAME=arm_api2_cont_test
IMAGE_NAME=arm_api2_img:humble

# Hook to the current SSH_AUTH_LOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run \
  -it \
  --network host \
  --privileged \
  --gpus all \
  --volume /dev:/dev \
  --volume /tmp/.x11-unix:/tmp/.x11-unix \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $CONTAINER_NAME \
  $IMAGE_NAME \
  /bin/bash
```
#### Some remarks: 
* `--network host` is used to enable docker container to access local PC network, i.e. see all ROS topics/services that local machine sees (from security perspective, potenital vulnerability)
* `--privileged` container has sudo privileges (i.e. can access all devices for example, cameras etc...)
* `--volume /dev:/dev` we map all local devices inside the container (to see cameras and sensors)
* `--gpus all` docker container has access to all GPUs that PC has (for ML stuff)
* `--volume /tmp/.x11-unix:/tmp/.x11-unix` remap X11 server for the GUI applications

#### Starting container

After `build --> run` we can use `start` command to start existing container on our PC. 

Use `docker start -i <cont_name>` to start container. 

#### Executing container 

If you want to access container that is already started in different bash (terminal), you can use: 
```
docker exec -it <cont_name> bash
```
#### Stopping container 

You can stop running container with:
```
docker stop <container_name> 
```

## Important remarks: 

If you stop container, everything you've changed will remain in your container. 
If you delete container, all progress will be lost. Therefore, delete container only if 
you've backed up your changes using GIT or something else. 

## In-depth Instructions 

Instructions on how to use following repository can be found [here](https://github.com/larics/docker_files/wiki).  
