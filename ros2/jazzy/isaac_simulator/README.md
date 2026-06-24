# Isaac Sim with ROS 2 Jazzy Docker

This Dockerfile sets up NVIDIA's Isaac Sim (4.2) integrated with ROS 2 Jazzy on Ubuntu 24.04 (Noble).

## Features

- **Isaac Sim 4.2**: Full NVIDIA simulation environment
- **ROS 2 Jazzy**: Latest ROS 2 distribution
- **GPU-Accelerated**: NVIDIA CUDA/RTX support
- **ROS 2 Integration**: Built-in Isaac ROS bridges
- **Python Development**: Jupyter, IPython, OpenCV ready
- **Display Support**: X11 forwarding for visualization
- **Development Tools**: CMake, Git, Build essentials

## Prerequisites

### Required
- **Docker**: Latest version
- **NVIDIA Docker Runtime**: For GPU access
  ```bash
  # Install nvidia-docker
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
  curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list
  sudo apt-get update && sudo apt-get install -y nvidia-docker2
  sudo systemctl restart docker
  ```

### Recommended
- **NVIDIA GPU**: RTX or professional GPU (simulation is GPU-accelerated)
- **NVIDIA CUDA 12.4+**: For optimal performance
- **8+ GB GPU VRAM**: For complex simulations

## Quick Start

### Option 1: Using docker-compose (Recommended)

```bash
cd /path/to/isaac_simulator

# Build the image
docker-compose build

# Run the container with GPU
docker-compose run --rm isaac-sim bash
```

### Option 2: Using the convenience script

```bash
cd /path/to/isaac_simulator
./run_docker.sh
```

### Option 3: Using Docker CLI directly

```bash
# Build
docker build -t isaac-sim-ros2:jazzy .

# Run with GPU support
docker run -it --rm \
  --runtime=nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/workspace:/root/workspace \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,video \
  isaac-sim-ros2:jazzy bash
```

## Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `INSTALL_ROS2` | `yes` | Install ROS 2 Jazzy |
| `INSTALL_ROS2_BRIDGE` | `yes` | Install Isaac ROS bridge packages |

## Building Custom Images

### Isaac Sim Only (no ROS 2)

```bash
docker build --build-arg INSTALL_ROS2=no -t isaac-sim:4.2 .
```

### Without ROS 2 Bridges

```bash
docker build --build-arg INSTALL_ROS2_BRIDGE=no -t isaac-sim-ros2:jazzy .
```

## Inside the Container

### Starting Isaac Sim

```bash
# Launch Isaac Sim GUI
isaac-sim.sh

# Or with native viewport
isaac-sim.sh --/renderer/active=rtx

# Run in headless mode
isaac-sim.sh --headless
```

### Testing the Installation

```bash
python3 test_isaac.py
```

### ROS 2 Integration

```bash
# Check ROS 2 installation
source /opt/ros/jazzy/setup.bash
ros2 --version

# Launch Isaac ROS bridge
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py
```

### Python Scripting

```bash
# Start Python interactive shell
python3

# Import Isaac modules
import omni
from omni.client import connect

# Create simulation
stage = omni.usd.get_context().get_stage()
```

### Jupyter Notebook

```bash
jupyter notebook --ip=0.0.0.0 --no-browser --allow-root
```

## Display/GUI Support

### On Linux with NVIDIA GPU

```bash
docker run -it --rm \
  --runtime=nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  -e NVIDIA_VISIBLE_DEVICES=all \
  isaac-sim-ros2:jazzy bash
```

### With docker-compose

Set your DISPLAY environment variable before running:

```bash
export DISPLAY=:0
docker-compose run --rm isaac-sim bash
```

### On macOS with XQuartz

```bash
xhost +local:
docker run -it --rm \
  --runtime=nvidia \
  -e DISPLAY=docker.for.mac.host.internal:0 \
  isaac-sim-ros2:jazzy bash
```

### Headless Mode (No Display)

```bash
# Run in background without display
docker run -it --rm \
  --runtime=nvidia \
  -v $(pwd)/workspace:/root/workspace \
  isaac-sim-ros2:jazzy \
  isaac-sim.sh --headless
```

## GPU Configuration

### Check GPU Inside Container

```bash
nvidia-smi
```

### Specify GPU Device

```bash
docker run -it --rm \
  --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=0 \
  isaac-sim-ros2:jazzy bash
```

### Multi-GPU Support

```bash
docker run -it --rm \
  --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  isaac-sim-ros2:jazzy bash
```

## Volume Mounting

### Workspace Directory

Mount your simulation projects:

```bash
docker run -it --rm \
  --runtime=nvidia \
  -v /path/to/your/projects:/root/workspace \
  isaac-sim-ros2:jazzy bash
```

### Isaac Projects Directory

Keep Isaac Sim projects persistent:

```bash
docker run -it --rm \
  --runtime=nvidia \
  -v /path/to/isaac/projects:/root/isaac_projects \
  isaac-sim-ros2:jazzy bash
```

## Performance Tuning

### CPU Limits

Control CPU usage:

```bash
docker run -it --rm \
  --runtime=nvidia \
  --cpus=4 \
  isaac-sim-ros2:jazzy bash
```

### Memory Limits

Limit container memory:

```bash
docker run -it --rm \
  --runtime=nvidia \
  --memory=16g \
  isaac-sim-ros2:jazzy bash
```

### GPU Memory

Set GPU memory allocation (in application):

```python
import omni.physx
# Configure PhysX GPU memory
```

## Common Workflows

### Developing ROS 2 Isaac Applications

```bash
docker run -it --rm \
  --runtime=nvidia \
  -v /path/to/ros2_ws:/root/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  isaac-sim-ros2:jazzy bash

# Inside container:
source /opt/ros/jazzy/setup.bash
cd /root/workspace
colcon build
source install/setup.bash
```

### Running Simulations in Batch

```bash
docker run --rm \
  --runtime=nvidia \
  -v $(pwd)/simulations:/root/workspace \
  isaac-sim-ros2:jazzy \
  python3 /root/workspace/simulation_runner.py
```

### Testing Isaac ROS Extensions

```bash
docker run -it --rm \
  --runtime=nvidia \
  -v $(pwd)/ros_extensions:/root/workspace \
  isaac-sim-ros2:jazzy bash

# Inside: test your extensions
```

## Troubleshooting

### Container Won't Start

Check docker and nvidia-docker installation:

```bash
docker --version
nvidia-docker --version
```

### No GPU Access

Verify nvidia-docker runtime:

```bash
docker run --rm --runtime=nvidia nvidia/cuda:12.4.0-runtime-ubuntu24.04 nvidia-smi
```

### Display Connection Issues

If X11 forwarding fails:

```bash
# Allow local connections
xhost +local:docker

# Or allow all (less secure)
xhost +
```

### Out of GPU Memory

Reduce simulation complexity or use CPU mode:

```bash
isaac-sim.sh --/renderer/active=cpu
```

### Performance Issues

- Check GPU utilization: `nvidia-smi -l 1`
- Reduce viewport resolution in settings
- Use headless mode for non-visual simulations
- Increase allocated CPU cores

### Module Import Errors

Ensure Isaac Sim environment is properly loaded:

```bash
# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Try direct import
python3 -c "import omni; print(omni.__file__)"
```

## Advanced Usage

### Custom Isaac Simulation Script

Create `my_simulation.py`:

```python
import omni
from omni.client import connect

async def setup_scene():
    # Create your simulation
    stage = omni.usd.get_context().get_stage()
    # ... add objects, physics, etc.
    await omni.client.list("/")

# Run simulation
omni.client.run_async(setup_scene)
```

Run it:

```bash
docker run --rm --runtime=nvidia \
  -v $(pwd)/my_simulation.py:/root/workspace/my_simulation.py \
  isaac-sim-ros2:jazzy \
  python3 /root/workspace/my_simulation.py
```

### ROS 2 Bridge with Isaac

```python
# ROS bridge integration example
from rclpy.node import Node
from geometry_msgs.msg import Pose

class IsaacROS2Bridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')
        # Initialize bridge
        pass
```

## Container Cleanup

Remove stopped containers:

```bash
docker-compose down
```

Remove image:

```bash
docker rmi isaac-sim-ros2:jazzy
```

Clean up all dangling resources:

```bash
docker system prune -a
```

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)

## License Notes

- **Isaac Sim**: NVIDIA proprietary (free for development/research)
- **ROS 2**: Apache 2.0
- **CUDA**: Requires NVIDIA GPU and driver

## Notes

- This image requires significant disk space (~30-50GB) due to Isaac Sim
- First run may take time to initialize Isaac Sim components
- GPU acceleration significantly improves simulation performance
- Headless mode is suitable for batch simulations and CI/CD pipelines

## Support & Feedback

For issues with:
- **Isaac Sim**: [NVIDIA Developer Forums](https://forums.developer.nvidia.com/c/omniverse/simulation/isaac-sim)
- **ROS 2**: [ROS Discourse](https://discourse.ros.org/)
- **Docker**: [Docker Community](https://www.docker.com/community)
