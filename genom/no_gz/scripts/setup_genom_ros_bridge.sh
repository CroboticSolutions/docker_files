#!/bin/bash
set -e

# Create catkin workspace if it doesn't exist
mkdir -p /root/catkin_ws/src
cd /root/catkin_ws/src

# Clone genom_ros_ctl
echo "Cloning genom_ros_ctl..."
git clone git@github.com:CroboticSolutions/genom_ros_ctl.git

# Clone genom_ros_bridge
echo "Cloning ros_gui_bridge..."
git clone git@github.com:CroboticSolutions/ros_gui_bridge.git

# Clone rospy_message_converter
echo "Cloning rospy_message_converter..."
git clone git@github.com:DFKI-NI/rospy_message_converter.git

echo "Building catkin_ws..."
cd /root/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
