#!/bin/bash

CONTAINER_NAME=gui_orchestrator_cont
IMAGE_NAME=gui_orchestrator_img:jazzy

# docker_files must be bind-mounted at the SAME absolute path it lives at on
# the host: launch_server's wizard drives docker compose/exec through the
# socket-mounted CLI, which is executed by the host's daemon, so relative
# build contexts and bind-mounts in the vendor_drivers compose file resolve
# against the host filesystem, not this container's.
DOCKER_FILES_DIR=/home/martin/Crobotics/docker_files

docker run \
  -d \
  --restart unless-stopped \
  --network host \
  --name $CONTAINER_NAME \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v $DOCKER_FILES_DIR:$DOCKER_FILES_DIR \
  -e VENDOR_DRIVERS_COMPOSE_FILE=$DOCKER_FILES_DIR/composers/vendor_drivers/docker-compose.yml \
  $IMAGE_NAME
