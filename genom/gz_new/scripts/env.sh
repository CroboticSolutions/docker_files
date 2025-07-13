export HOME=/root
export ROBOTPKG_BASE=$HOME/openrobots

export PATH=$PATH:$ROBOTPKG_BASE/bin:$ROBOTPKG_BASE/sbin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG_BASE/lib/pkgconfig

export DEVEL_BASE=$HOME/devel
export LD_LIBRARY_PATH=$DEVEL_BASE/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$DEVEL_BASE/lib/genom/ros/plugins:$LD_LIBRARY_PATH

export PATH=$PATH:$DEVEL_BASE/bin:$DEVEL_BASE/sbin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$DEVEL_BASE/lib/pkgconfig

export GAZEBO_MODEL_PATH=$ROBOTPKG_BASE/share/gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$ROBOTPKG_BASE/share/gazebo/models/air-gazebo-resources:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$DEVEL_BASE/workspaces/minithex-cntrl-python-scripts/gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$ROBOTPKG_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$DEVEL_BASE/lib/gazebo:${GAZEBO_PLUGIN_PATH}

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEVEL_BASE/share:$ROBOTPKG_BASE/share
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$DEVEL_BASE/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python3.10/site-packages

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$HOME/openrobots/share

