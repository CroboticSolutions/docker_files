#!/bin/bash

# Isaac Sim with ROS 2 Jazzy Docker Runner Script

set -e

IMAGE_NAME="isaac-sim-ros2:jazzy"
CONTAINER_NAME="isaac_sim_ros2_dev"

echo "======================================"
echo "Isaac Sim + ROS 2 Jazzy Docker Setup"
echo "======================================"

# Check if NVIDIA Docker runtime is available
if ! command -v nvidia-docker &> /dev/null; then
    echo "⚠ Warning: nvidia-docker not found"
    echo "  Make sure NVIDIA Docker runtime is installed"
    echo "  See: https://github.com/NVIDIA/nvidia-docker"
fi

# Check NVIDIA GPU
if ! nvidia-smi &> /dev/null; then
    echo "⚠ Warning: NVIDIA GPU not available"
    echo "  Isaac Sim will run in headless mode"
fi

echo ""
echo "Building Isaac Sim Docker image..."
docker-compose build

echo ""
echo "Starting Isaac Sim container..."
docker-compose run --rm isaac-sim bash

echo ""
echo "Container stopped."
