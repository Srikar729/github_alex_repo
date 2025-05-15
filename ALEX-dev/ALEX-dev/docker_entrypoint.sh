#!/bin/bash
set -e

# setup ros2 environment
export ROS_DOMAIN_ID=101 
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/ros2_ws/install/setup.bash"

exec "$@"
