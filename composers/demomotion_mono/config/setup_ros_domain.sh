# DDS domain for this workspace (Piper / arms_ws). Keeps discovery off ROS_DOMAIN_ID=0
# where unrelated robots (abb1, etc.) often appear.
#
# Per-terminal override before sourcing install/setup.bash:
#   export ROS_DOMAIN_ID=7
#
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-52}"
