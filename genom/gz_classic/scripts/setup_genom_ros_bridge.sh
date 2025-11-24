

# Clone genom_ros_ctl
echo "Cloning genom_ros_ctl..."
cd /root/catkin_ws/src
git clone git@github.com:CroboticSolutions/genom_ros_ctl.git

# Clone genom_ros_bridge
echo "Cloning genom_ros_bridge..."
cd /root/catkin_ws/src
git clone git@github.com:CroboticSolutions/ros_gui_bridge.git

echo "Building catkin_ws..."
cd /root/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release