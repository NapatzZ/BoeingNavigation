#!/bin/bash

source /opt/ros/humble/setup.bash

touch /tmp/.build_complete

if [ -f /root/skuba_ws/install/setup.bash ]; then
  source /root/skuba_ws/install/setup.bash
fi


# Build the mounted workspace so `ros2 launch` can find the packages.
# Set SKIP_BUILD=1 to bypass (fast restarts when nothing changed).
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  ( cd /root/skuba_ws && colcon build ) \
    || echo "[entrypoint] colcon build reported errors; continuing anyway."
fi

if [ -f /root/skuba_ws/install/setup.bash ]; then
  source /root/skuba_ws/install/setup.bash
fi

# Run whatever compose passes as `command:` (default CMD is bash).
exec "$@"
