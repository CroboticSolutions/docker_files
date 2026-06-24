# MuJoCo Simulator with ROS 2 Jazzy Docker

This Dockerfile sets up a complete development environment for MuJoCo physics simulator integrated with ROS 2 Jazzy on Ubuntu Noble.

## Features

- **MuJoCo 3.1.0**: Latest physics simulation engine
- **ROS 2 Jazzy**: Full ROS 2 Jazzy Desktop distribution
- **Python 3 Virtual Environment**: Isolated Python environment with common tools
- **PyMuJoCo**: Python bindings for MuJoCo
- **Visualization Tools**: Matplotlib, Jupyter for interactive development
- **GPU Support**: Ready for NVIDIA GPU acceleration
- **GUI Support**: X11 forwarding for visualization

## Quick Start

### Option 1: Using docker-compose (Recommended)

```bash
cd /path/to/mujoco_simulator

# Build the image
docker-compose build

# Run the container
docker-compose run --rm mujoco bash
```

### Option 2: Using the convenience script

```bash
cd /path/to/mujoco_simulator
./run_docker.sh
```

### Option 3: Using Docker CLI directly

```bash
# Build
docker build -t ros2-mujoco:jazzy .

# Run with X11 display forwarding
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/workspace:/root/workspace \
  --gpus all \
  ros2-mujoco:jazzy bash
```

## Build Arguments

The Dockerfile supports the following build arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `INSTALL_ROS2` | `yes` | Install ROS 2 Jazzy (set to `no` to skip) |
| `MUJOCO_VERSION` | `3.1.0` | MuJoCo version to install |
| `PY_VENV` | `/root/py_global` | Python virtual environment path |

## Usage Examples

### Building with Custom MuJoCo Version

```bash
docker build --build-arg MUJOCO_VERSION=3.0.0 -t ros2-mujoco:jazzy .
```

### Building without ROS 2 (MuJoCo only)

```bash
docker build --build-arg INSTALL_ROS2=no -t mujoco-only:latest .
```

## Inside the Container

Once inside the container, you can:

```bash
# Test MuJoCo installation
python3 -c "import mujoco; print(mujoco.__version__)"

# Access ROS 2 (if installed)
source /opt/ros/jazzy/setup.bash
ros2 --version

# Start Jupyter for interactive development
jupyter notebook --ip=0.0.0.0 --no-browser

# Run Python scripts with MuJoCo
python3 my_simulation.py
```

## Display/GUI Support

For X11 display forwarding (visualization):

### On Linux:

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/.Xauthority:/root/.Xauthority:rw \
  ros2-mujoco:jazzy bash
```

### On macOS (with XQuartz):

```bash
# Install XQuartz first, then:
xhost +local:
docker run -it --rm \
  -e DISPLAY=docker.for.mac.host.internal:0 \
  ros2-mujoco:jazzy bash
```

## GPU Support

If you have NVIDIA GPU and nvidia-docker installed:

```bash
docker run -it --rm \
  --gpus all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
  ros2-mujoco:jazzy bash
```

Or with docker-compose (uncomment the `runtime: nvidia` line):

```yaml
runtime: nvidia
```

## Workspace Volume

Mount your workspace directory to `/root/workspace`:

```bash
docker run -it --rm \
  -v /path/to/your/workspace:/root/workspace \
  ros2-mujoco:jazzy bash
```

Then work directly in `/root/workspace` inside the container.

## Testing the Installation

### Quick Test Script

Create a file `test_mujoco.py`:

```python
#!/usr/bin/env python3
import mujoco
import numpy as np

# Load a model
model = mujoco.MjModel.from_xml_string("""
<mujoco>
  <worldbody>
    <body name="ball" pos="0 0 1">
      <joint name="ball_x" type="slide" axis="1 0 0" range="-1 1"/>
      <geom name="ball" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
""")

# Create data
data = mujoco.MjData(model)

# Step simulation
mujoco.mj_step(model, data)

print(f"✓ MuJoCo {mujoco.__version__} is working!")
print(f"  Ball position: {data.body('ball').xpos}")
```

Run it inside the container:

```bash
python3 test_mujoco.py
```

## Common Issues

### X11 Display Connection Refused

Make sure X server permissions are set:

```bash
xhost +local:docker
```

### GPU Not Available

Verify NVIDIA Docker runtime:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-runtime-ubuntu24.04 nvidia-smi
```

### MuJoCo License

MuJoCo is now free and open-source (Apache 2.0). No license key needed.

## Performance Tuning

For better simulation performance:

1. Mount workspace with `cached` option: `-v /path:/root/workspace:cached`
2. Use `--cpus` limit to allocate CPU cores: `--cpus=4`
3. For GPU-intensive tasks, ensure NVIDIA Docker runtime is properly configured

## Cleanup

To stop and remove the container:

```bash
docker-compose down
```

Or to remove the image:

```bash
docker rmi ros2-mujoco:jazzy
```

## Additional Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [dm-control Tutorial](https://github.com/deepmind/dm_control)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## Troubleshooting

### Container exits immediately

Check logs:

```bash
docker-compose logs mujoco
```

### Virtual environment not activated

Add to your `.bashrc` or run manually:

```bash
source /root/py_global/bin/activate
```

### Python packages not found

Make sure virtual environment is activated:

```bash
source /root/py_global/bin/activate
pip list  # Should show mujoco, numpy, etc.
```
