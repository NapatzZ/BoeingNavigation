# Boeing Navigation Exercises

A hands-on course for learning SLAM and Nav2 on the Boeing robot
(Kobuki base + RPLIDAR A3, ROS 2 Humble). Work through the exercises in order;
each one builds on the previous one.

| # | Exercise | You will learn | Time |
|---|----------|----------------|------|
| 0 | [Bring-up check](00_bringup_check.md) | Robot, TF tree, sensors, safe teleop | 30 min |
| 1 | [Mapping with SLAM](01_slam_mapping.md) | Drive the robot and build + save a map | 1 h |
| 2 | [Navigating on a saved map](02_navigation_saved_map.md) | Localisation (AMCL), sending goals | 1 h |
| 3 | [Tuning after mapping](03_tuning.md) | Costmaps, planner, controller, AMCL tuning | 2-3 h |
| 4 | [Online SLAM + Nav2 together](04_slam_and_nav_together.md) | Navigate while the map is still being built | 1-2 h |
| 5 | [Named waypoints](05_waypoints.md) | Save places and send goals by name, routes and patrol | 1-2 h |

## Packages used

| Package | Role |
|---------|------|
| `boeing_description` | URDF and meshes (`base_link` -> `lidar_link` -> `laser`) |
| `boeing_bringup` | Starts `robot_state_publisher`, `kobuki_node` and the RPLIDAR A3 |
| `boeing_navigation` | slam_toolbox + Nav2 launch files and parameters |

Files you will touch:

```
boeing_bringup/config/kobuki_node.yaml      # base parameters
boeing_bringup/config/sllidar_a3.yaml       # lidar parameters
boeing_navigation/config/slam_toolbox.yaml  # SLAM parameters
boeing_navigation/config/nav2_params.yaml   # Nav2 parameters (exercise 3)
boeing_navigation/maps/                     # your saved maps and waypoints
boeing_navigation/scripts/waypoint_manager.py  # exercise 5
```

## Before you start

1. **udev rules installed on the host** (see the top-level `README.md`), so that
   `/dev/kobuki` and `/dev/rplidar` exist.
2. **Container running with the USB devices passed through** (e.g. `--device /dev/kobuki
   --device /dev/rplidar`, or `--privileged`) and with network access to your laptop
   if you run RViz there.
3. **Workspace is built** and sourced (`~/.bashrc` already sources `/ros_ws/install/setup.bash`).
   The image builds with `--symlink-install`, so editing an existing YAML under `config/`
   takes effect on the next launch without rebuilding. *New* files need `colcon build`.
4. **Visualisation:** RViz needs a display. Run it on a laptop on the same network, or use
   Foxglove (`foxglove_bridge` is installed in the image).
5. **Safety:** keep a hand on the robot's power switch / be ready to lift it. Start every
   new behaviour at low speed in an open area.

## Terminal conventions

The exercises use numbered terminals. Open each in the container
(`docker exec -it <container> bash`, or use `byobu` which is installed):

- **T1** - robot bring-up (always running)
- **T2** - SLAM or navigation
- **T3** - RViz
- **T4** - teleop / helper commands

## Handing in work

For each exercise, fill in the **Deliverables** at the bottom and keep screenshots
and the saved files in `boeing_navigation/maps/` and a `notes/` folder of your own.
