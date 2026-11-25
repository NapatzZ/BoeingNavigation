# skuba-navigation POC (TurtleBot + Kobuki + RPLIDAR)

Minimal Docker setup to run Nav2 with SKUBA params on a TurtleBot.

## Quick Start

```bash
# 1. Clone POC branch
git clone -b skuba-navigation-poc https://github.com/NapatzZ/BoeingNavigation.git
cd BoeingNavigation

# 2. Put sibling packages next to this repo (or edit paths in docker-compose.poc.yml)
#    /Users/nimbuss/skuba_bringup
#    /Users/nimbuss/skuba_interfaces
#    /Users/nimbuss/skuba_description

# 3. Build & start
docker compose -f docker-compose.poc.yml up -d

# 4. Enter container
docker exec -it skuba_nav_poc bash
```

## Inside Container

```bash
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# SLAM (mapping)
ros2 launch skuba_navigation nav2_bringup_launch.py slam:=True publish_laser_tf:=false

# Localization (with existing map)
ros2 launch skuba_navigation nav2_bringup_launch.py slam:=False map:=maps/30_skuba_home/skuba_home.yaml publish_laser_tf:=false
```

## Tune Nav2 Params

1. Edit `params/rcj_param.yaml` in this repo
2. Rebuild container:
   ```bash
   docker compose -f docker-compose.poc.yml build --no-cache
   docker compose -f docker-compose.poc.yml up -d
   ```

Key params to tune:
- `controller_server.FollowPath.max_vel_x` — max linear speed
- `controller_server.FollowPath.max_vel_theta` — max angular speed
- `local_costmap.local_costmap.inflation_radius` — obstacle padding
- `amcl.max_particles` / `min_particles` — localization accuracy vs CPU
- `bt_navigator.default_nav_to_pose_bt_xml` — behavior tree

## Hardware Notes

| Device | Mount | Notes |
|--------|-------|-------|
| Kobuki base | `/dev/bus/usb` | USB serial |
| RPLIDAR | `/dev:/dev` | `/dev/ttyUSB*` |
| No RealSense, no joystick | — | Removed for POC |

## URDF

The launch does **not** load `robot_description`. It publishes a static `base_link → laser` TF (hardcoded offset). Set `publish_laser_tf:=false` to use TurtleBot's own URDF laser joint.