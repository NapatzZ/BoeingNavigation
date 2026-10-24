#!/bin/bash

source /opt/ros/humble/setup.bash

# Zenoh router -- the image's middleware is rmw_zenoh (RMW_IMPLEMENTATION), and
# rmw_zenoh nodes meet through one Zenoh router on tcp/localhost:7447 (host
# network: one router for the whole robot). Every SKUBA container runs this
# same watcher: whichever holds port 7447 is the router, the rest stand by and
# take over within ~2 s if it goes away. Nodes that have met keep talking
# directly, so traffic does not stop with the router; only nodes that start
# while no router is up wait for one (they retry every <=4 s). A router that
# hangs (deadlock, SIGSTOP) still holds the port, so nobody would take over:
# the container running it watches that connections to 7447 get accepted --
# the stand-by checks keep knocking -- and kills it once some have waited 3
# checks (~6 s) in a row. Started before the build, so the router is up early.
# ZENOH_ROUTER=0 turns it off here. Log: /tmp/zenoh_router.log.
if [ "${ZENOH_ROUTER:-1}" != "0" ] && [ "${RMW_IMPLEMENTATION:-}" = rmw_zenoh_cpp ]; then
  (
    set +e  # a router that exits (port taken, crash) must not end the watcher
    queued() {  # connections waiting to be accepted on 7447 (from /proc: not every image has ss)
      local q=0 l st qs
      while read -r _ l _ st qs _; do
        [ "${st}" = 0A ] && [ "${l##*:}" = 1D17 ] && q=$((q + 16#${qs#*:}))
      done < <(tail -n +2 -q /proc/net/tcp /proc/net/tcp6 2>/dev/null)
      echo "${q}"
    }
    while true; do
      if ! (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null; then
        echo "[zenoh] $(date +%T) router on tcp/localhost:7447 from navigation"
        /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd &
        router=$! stuck=0
        while sleep 2 && kill -0 "${router}" 2>/dev/null; do
          # Read the queue before knocking: a knock is accepted within the next 2 s.
          if [ "$(queued)" -gt 0 ]; then stuck=$((stuck + 1)); else stuck=0; fi
          if [ "${stuck}" -ge 3 ]; then
            echo "[zenoh] $(date +%T) router stopped accepting -- killed, the stand-by takes over"
            kill -9 "${router}"
          fi
          (exec 3<>/dev/tcp/127.0.0.1/7447) 2>/dev/null
        done
        wait "${router}" 2>/dev/null
      fi
      sleep 2
    done
  ) > /tmp/zenoh_router.log 2>&1 &
fi

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
