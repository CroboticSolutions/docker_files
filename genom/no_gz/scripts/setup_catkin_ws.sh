#!/bin/bash
set -e

echo "Setting up catkin_ws for UAV (no Gazebo)..."

# Create catkin workspace
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/src

# Clone geometry (tf, eigen_conversions)
echo "Cloning geometry..."
git clone https://github.com/ros/geometry.git

# Clone geometry2 (tf2)
echo "Cloning geometry2..."
git clone https://github.com/ros/geometry2.git

# Clone angles
echo "Cloning angles..."
git clone https://github.com/ros/angles.git
cd angles
git checkout noetic-devel
cd ..

# Clone diagnostics
echo "Cloning diagnostics..."
git clone https://github.com/ros/diagnostics.git
cd diagnostics
git checkout noetic-devel
cd ..

# Install orocos-kdl (required for tf)
echo "Installing liborocos-kdl-dev..."
apt-get install -y liborocos-kdl-dev

# Build catkin workspace
echo "Building catkin_ws..."
cd $HOME/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "Catkin workspace setup complete!"
echo "Source the workspace with: source $HOME/catkin_ws/devel/setup.bash"
