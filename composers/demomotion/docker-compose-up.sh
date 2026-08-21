#!/bin/bash

# Hook to the current SSH_AUTH_SOCK - since it changes
# https://www.talkingquickly.co.uk/2021/01/tmux-ssh-agent-forwarding-vs-code/
ln -sf $SSH_AUTH_SOCK ~/.ssh/ssh_auth_sock

# perception clones private repos over BuildKit SSH mounts
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# Allow X11 connections from Docker
xhost +local:docker

# Run docker-compose with any passed arguments
docker compose "$@"
