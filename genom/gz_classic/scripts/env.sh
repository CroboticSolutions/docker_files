# Init path
export HOME=/root
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/openrobots/bin:$HOME/openrobots/sbin

# Set env variables
export PYTHONPATH=$PYTHONPATH:$HOME/openrobots/lib/python3.10/site-packages

# Sources for robotpkg and devel folders
export DEVEL_BASE=$HOME/devel
export ROBOTPKG_BASE=$HOME/openrobots

unset GAZEBO_PLUGIN_PATH
unset GAZEBO_MODEL_PATH

# Location of gazebo related resources
export GAZEBO_MODEL_PATH=$ROBOTPKG_BASE/share/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$ROBOTPKG_BASE/share/gazebo/models/air-gazebo-resources:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$DEVEL_BASE/minithex-cntrl-python-scripts/gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$ROBOTPKG_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$DEVEL_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}

export PATH=$PATH:$ROBOTPKG_BASE/bin:$ROBOTPKG_BASE/sbin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG_BASE/lib/pkgconfig

export LD_LIBRARY_PATH=$DEVEL_BASE/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$DEVEL_BASE/lib/genom/ros/plugins:$LD_LIBRARY_PATH

export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$DEVEL_BASE/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python3.10/site-packages

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$HOME/openrobots/share

# Tmuxinator editing 
export EDITOR="nano"
export SHELL="bash"

# Source ROS
source /root/openrobots/etc/ros/setup.bash
source $HOME/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEVEL_BASE/share:$ROBOTPKG_BASE/share:/root/catkin_ws/src/
export GAZEBO_PLUGIN_PATH=$ROBOTPKG_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$DEVEL_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$HOME/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}

