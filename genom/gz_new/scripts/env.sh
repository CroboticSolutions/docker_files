# Init path
export HOME=/root
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$HOME/openrobots/bin:$HOME/openrobots/sbin

# Set env variables
export PYTHONPATH=$PYTHONPATH:$HOME/openrobots/lib/python3.10/site-packages
# Used if the old gazebo is used, new gazebo has GZ_SIM_RESOURCE_PATH and GZ_SIM_SYSTEM_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH=$HOME/openrobots/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=$HOME/openrobots/share/gazebo/models
export DEVEL_BASE=$HOME/devel

export ROBOTPKG_BASE=$HOME/openrobots

export PATH=$PATH:$ROBOTPKG_BASE/bin:$ROBOTPKG_BASE/sbin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG_BASE/lib/pkgconfig

export DEVEL_BASE=$HOME/devel
export LD_LIBRARY_PATH=$DEVEL_BASE/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$DEVEL_BASE/lib/genom/ros/plugins:$LD_LIBRARY_PATH

export PATH=$PATH:$DEVEL_BASE/bin:$DEVEL_BASE/sbin
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$DEVEL_BASE/lib/pkgconfig

export GZ_SIM_RESOURCE_PATH=$ROBOTPKG_BASE/share/gazebo/models/
export GZ_SIM_RESOURCE_PATH=$ROBOTPKG_BASE/share/gazebo/models/air-gazebo-resources:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$DEVEL_BASE/minithex-cntrl-python-scripts/gazebo/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/openrobots/lib/gazebo/
export GZ_SIM_SYSTEM_PLUGIN_PATH=$DEVEL_BASE/lib/gazebo/:$GZ_SIM_SYSTEM_PLUGIN_PATH

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEVEL_BASE/share:$ROBOTPKG_BASE/share
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$DEVEL_BASE/lib/python3.10/site-packages
export PYTHONPATH=$PYTHONPATH:$ROBOTPKG_BASE/lib/python3.10/site-packages

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$HOME/openrobots/share
