#!/bin/bash
set -e

echo "Setting up catkin_ws..."

# Create catkin workspace
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/src

# Clone control_toolbox
echo "Cloning control_toolbox..."
git clone https://github.com/ros-controls/control_toolbox.git
cd control_toolbox
git checkout noetic-devel
cd ..

# Clone control_msgs
echo "Cloning control_msgs..."
git clone https://github.com/ros-controls/control_msgs.git
cd control_msgs
git checkout kinetic-devel
cd ..

# Clone dynamic_reconfigure
echo "Cloning dynamic_reconfigure..."
git clone https://github.com/ros/dynamic_reconfigure

# Clone realtime_tools
echo "Cloning realtime_tools..."
git clone https://github.com/ros-controls/realtime_tools.git
cd realtime_tools
git checkout noetic-devel
cd ..

# Clone ros_control
echo "Cloning ros_control..."
git clone https://github.com/ros-controls/ros_control.git
cd ros_control
git checkout noetic-devel
cd ..

# Clone urdf
echo "Cloning urdf..."
git clone https://github.com/ros/urdf.git

# Clone geometry
echo "Cloning geometry..."
git clone https://github.com/ros/geometry.git

# Clone angles
echo "Cloning angles..."
git clone https://github.com/ros/angles.git
cd angles
git checkout noetic-devel
cd ..

# Clone geometry2
echo "Cloning geometry2..."
git clone https://github.com/ros/geometry2.git

# Clone resource_retriever
echo "Cloning resource_retriever..."
git clone https://github.com/ros/resource_retriever.git
cd resource_retriever
git checkout noetic-devel
cd ..

# Clone rosconsole_bridge
echo "Cloning rosconsole_bridge..."
git clone https://github.com/ros/rosconsole_bridge.git

# Clone nodelet_core
echo "Cloning nodelet_core..."
git clone https://github.com/ros/nodelet_core.git

# Clone bond_core
echo "Cloning bond_core..."
git clone https://github.com/ros/bond_core.git
cd bond_core
git checkout noetic-devel
cd ..

# Clone image_common
echo "Cloning image_common..."
git clone https://github.com/ros-perception/image_common.git
cd image_common
git checkout noetic-devel
cd ..

# Clone vision_opencv
echo "Cloning vision_opencv..."
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git checkout noetic
cd ..

# Clone diagnostics
echo "Cloning diagnostics..."
git clone https://github.com/ros/diagnostics.git
cd diagnostics
git checkout noetic-devel
cd ..

# Install orocos-kdl
echo "Installing liborocos-kdl-dev..."
apt-get install -y liborocos-kdl-dev

# Build and install OpenCV 4.x
echo "Building OpenCV 4.x..."
cd $HOME
apt-get update && apt-get install -y cmake g++ wget unzip
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
cd opencv-4.x
mkdir -p build
cd build
cmake ..
cmake --build . -j$(nproc)
make install
cd $HOME

# Build and install yaml-cpp 0.6.2
echo "Building yaml-cpp 0.6.2..."
cd $HOME
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
git checkout tags/yaml-cpp-0.6.2
mkdir -p build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc)
make install
cd $HOME

# Build catkin workspace
echo "Building catkin_ws..."
cd $HOME/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

echo "Catkin workspace setup complete!"
echo "Source the workspace with: source $HOME/catkin_ws/devel/setup.bash"
