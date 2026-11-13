#!/bin/bash
set -e

# Source ROS 2 and the workspace (if built) for any command run in the container.
source /opt/ros/humble/setup.bash
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

exec "$@"
