# Init path
export HOME=/root
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/openrobots/bin:$HOME/openrobots/sbin

# Set env variables
export PYTHONPATH=$PYTHONPATH:$HOME/openrobots/lib/python3.10/site-packages

# Sources for robotpkg and devel folders
export DEVEL_BASE=$HOME/devel
export ROBOTPKG_BASE=$HOME/openrobots

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

# Source catkin_ws if it exists
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    source $HOME/catkin_ws/devel/setup.bash
    export ROS_PACKAGE_PATH=$DEVEL_BASE/share:$ROBOTPKG_BASE/share:$HOME/catkin_ws/src/
else
    export ROS_PACKAGE_PATH=$DEVEL_BASE/share:$ROBOTPKG_BASE/share
fi
