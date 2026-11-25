#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build the workspace (including mounted packages) if not already built
# Set SKIP_BUILD=1 to bypass (fast restarts when nothing changed)
if [ "${SKIP_BUILD:-0}" != "1" ] && [ ! -f /tmp/.build_complete ]; then
    echo "[entrypoint] Building workspace..."
    ( cd /ros_ws && colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release ) \
        || echo "[entrypoint] colcon build reported errors; continuing anyway."
    touch /tmp/.build_complete
fi

# Source workspace if built
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

exec "$@"