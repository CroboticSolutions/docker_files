#!/bin/bash

CONTAINER_NAME=arm_api2_cont_izzy_twist
IMAGE_NAME=arm_api2_img:izzy_twist

# Hook to the current SSH_AUTH_SOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

docker run \
  -it \
  --network host \
  --privileged \
  --gpus all \
  --volume /dev:/dev \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume ~/.ssh/ssh_auth_sock:/ssh-agent \
  --env SSH_AUTH_SOCK=/ssh-agent \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $CONTAINER_NAME \
  $IMAGE_NAME \
  /bin/bash
