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

# Foxglove Bridge -- the robot's one monitoring endpoint: Foxglove Studio on
# ws://<robot>:8765 shows every subsystem (one bridge sees the whole ROS graph).
# Every SKUBA container runs this same watcher; whichever holds port 8765
# serves it, the rest stand by and take over within ~10 s if it goes away.
# FOXGLOVE=0 turns it off here. Background only: it never delays this
# container's command. An image built before foxglove_bridge was added gets it
# apt-installed once (needs network). Log: /tmp/foxglove_bridge.log.
if [ "${FOXGLOVE:-0}" != "0" ]; then
  (
    set +e  # a bridge that exits (port taken, crash) must not end the watcher
    port="${FOXGLOVE_PORT:-8765}"
    if [ ! -d /opt/ros/humble/share/foxglove_bridge ]; then
      echo "[foxglove] ros-humble-foxglove-bridge not in this image; installing..."
      apt-get update -qq && apt-get install -y -qq --no-install-recommends ros-humble-foxglove-bridge
    fi
    while true; do
      if ! (exec 3<>"/dev/tcp/127.0.0.1/${port}") 2>/dev/null; then
        echo "[foxglove] $(date +%T) serving ws://<robot>:${port} from navigation"
        ros2 run foxglove_bridge foxglove_bridge --ros-args \
          -r __node:=foxglove_bridge_navigation -p port:="${port}"
      fi
      sleep 10
    done
  ) > /tmp/foxglove_bridge.log 2>&1 &
  echo "[entrypoint] foxglove: ws://<robot>:${FOXGLOVE_PORT:-8765} (FOXGLOVE=1 to turn on)"
fi

# Run whatever compose passes as `command:` (default CMD is bash).
exec "$@"
