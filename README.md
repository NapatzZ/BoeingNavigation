# skuba-navigation

Nav2-based navigation for the SKUBA robot (ROS 2 Humble).

The robot model lives in `skuba_description`; hardware bringup
(kobuki, LiDAR, teleop, robot_state_publisher) lives in
`skuba_bringup`. Both nav launch files include the bringup launch,
so you do not start them separately.

## Launch files

There are two task launch files. Pick one — never run both.

### 1. `cheesecake_nav.launch.py` — AMCL localization

The normal stack: hardware, nav2 with AMCL, RViz, and the service nodes.

```bash
ros2 launch skuba_navigation cheesecake_nav.launch.py
ros2 launch skuba_navigation cheesecake_nav.launch.py map:=/path/to/map.yaml
```

| Argument | Default |
| --- | --- |
| `map` | `maps/30_skuba_home/skuba_home.yaml` |
| `params_file` | `params/rcj_param.yaml` |

### 2. `cheesecake_resturant.launch.py` — AMCL localization, restaurant map

For the restaurant task. Same AMCL-based localization as `cheesecake_nav.launch.py`
(via `nav2_bringup_launch.py`'s stock `localization_launch.py`), just defaulted
to the restaurant map and restaurant params file, plus `following_server`.

```bash
ros2 launch skuba_navigation cheesecake_resturant.launch.py
```

| Argument | Default |
| --- | --- |
| `map` | `maps/20_restaurant/resturant_map_long.yaml` |
| `params_file` | `params/nav2_params.yaml` |

### Building a new map (SLAM Toolbox)

`nav2_slam_bringup_launch.py` is the mapping entry point. Nothing includes it —
you run it by hand, drive the robot around, then save.

```bash
ros2 launch skuba_navigation nav2_slam_bringup_launch.py
ros2 run skuba_navigation map_save_cli        # interactive save helper
```

It runs `slam_toolbox`'s own `online_async_launch.py` (with
`params/slam_params.yaml` as `slam_params_file` — `mode: mapping`,
loop closure on, consuming the raw `/scan`), nav2's `navigation_launch.py` (with
`params/nav2_params.yaml` as `params_file`), and RViz. It also brings up `filter_lidar`,
`emergency_stop`, and `twist_mux` directly (unlike the two task launches,
which get these via `nav2_bringup_launch.py`), so teleop driving during a
mapping session reaches the robot through the same `cmd_vel` arbitration and
software e-stop as the task launches use.

`map_save_cli` prompts for a map name, an optional category folder (e.g.
`30_skuba_home` — pick an existing one to add to it, or a new `NN_name` to
start one), and the maps root (default `maps/`), then writes directly into
`maps/<category>/<name>.{pgm,yaml,data,posegraph}` — matching the existing
layout, not a further per-map subfolder. `setup.py` installs the whole `maps/`
tree, so a new category folder is picked up on the next `colcon build` with
no edits.

### Internal launch files

Not meant to be run directly, but they are valid standalone launches:

- `nav2_bringup_launch.py` — nav2 core (localization + navigation), plus
  `emergency_stop`, `filter_lidar`, `twist_mux`, and the `base_link -> laser`
  static transform. Included by both task launches. `use_localization:=False`
  skips AMCL (used by `nav2_slam_bringup_launch.py`'s mapping stack, which
  runs SLAM Toolbox instead of AMCL for `map -> odom`, and brings up
  `filter_lidar`/`emergency_stop`/`twist_mux` itself instead of going through
  this file).

## Nodes

Executables registered in `setup.py`:

| Executable | Purpose |
| --- | --- |
| `navigation_server` | All base-motion actions + location/costmap services (see below) |
| `save_location` | Writes the current pose into `share/saved_pose.yaml` |
| `door_detector` | Reports whether the door ahead is open |
| `costmap_reconfig_server` | Applies costmap presets from `params/costmap_dynamic_config.yaml` at runtime |
| `following_server` | Runs `behavior_trees/follow_point.xml` to follow a tracked person |
| `emergency_stop` | Publishes the twist_mux e-stop lock |
| `filter_lidar` | Republishes `/scan` as `/scan_filtered` with an optional angular mask |
| `person_cluster_filter` | Strips person-sized LiDAR clusters into `/scan_slam` (not launched by default) |
| `map_save_cli` | Interactive prompt to serialize the SLAM Toolbox map to disk |

`cmd_vel` arbitration is handled by `twist_mux` (`params/twist_mux.yaml`): joy,
keyboard, follow and nav sources are muxed into the single `/cmd_vel` the base
drives, with `emergency_stop` holding the lock topic.

## navigation_server

Every way of moving the base is an **action** under `/skuba/nav/`, so a caller
(YASMIN today, py_trees_ros next) gets feedback, can cancel its own goal, and
learns why a goal failed. Only instant operations remain services.
**Full reference: [docs/navigation_server.md](docs/navigation_server.md)**
(goal / result / feedback fields, error codes, cancel, parameters, code
layout, migration from `nav_to_pose`).

| Action | Type | Backend | Replaces (`nav_to_pose` service) |
| --- | --- | --- | --- |
| `go_to_pose` | `GoToPose` | Nav2 | `nav_to_position` |
| `go_to_location` | `GoToLocation` | Nav2, saved location | `nav_to_name` |
| `go_to_frame` | `GoToFrame` | Nav2 + heading correction, standoff next to a TF | `nav_to_tf` |
| `rotate` | `RotateInPlace` | PD controller on `cmd_vel_nav_manual` | `rotate` |
| `drive_straight` | `DriveStraight` | closed loop on `cmd_vel_nav_manual` | `move_forward` |
| `nudge_to_frame` | `NudgeToFrame` | turn / drive / face on `cmd_vel_nav_manual` | `nudge_to_tf` |

| Service | Type | Replaces |
| --- | --- | --- |
| `clear_costmaps` | `std_srvs/Trigger` | `clear_costmap` |
| `save_location` | `SaveLocationCommand` | `save_pose` |
| `reload_locations` | `std_srvs/Trigger` | `reload_yaml` |

`cancel_goal` is gone: cancel through the action itself (the caller's goal
handle, or `ros2 service call /skuba/nav/<action>/_action/cancel_goal
action_msgs/srv/CancelGoal "{}"` to cancel all goals on that action).
A new goal on any of the six actions preempts the running one.

```bash
ros2 action send_goal --feedback /skuba/nav/go_to_location skuba_interfaces/action/GoToLocation "{location: kitchen}"
```

## Services of the other nodes

All under `/skuba/nav/`.

| Service | Node |
| --- | --- |
| `save_location_with_side` (`SaveLocationWithSide`) | `save_location` |
| `door_detector/toggle` | `door_detector` |
| `costmap_reconfig`, `reset_costmap_config`, `controller_reconfig` | `costmap_reconfig_server` |
| `enable_backward_mode`, `disable_backward_mode` | `costmap_reconfig_server` |
| `toggle_following` (`std_srvs/SetBool`) | `following_server` |
| `filter_lidar` | `filter_lidar` |

The person-following service `/skuba/cheesecake/nav/following` is **not** in
this package — it is served by `skuba_vision`'s `follower` node (different
namespace prefix, unaffected by this package's renames).

## Docker

Compose lives in `docker/`, not the package root:

```bash
cd docker
docker compose up -d
docker exec -it skuba-nav bash
```

Runs `napatku/skuba-navigation:latest` (built from `docker/dockerfile`, based on
`osrf/ros:humble-desktop-full`) with `--network host`, `ROS_DOMAIN_ID=10`,
`privileged`, and USB passthrough for the LiDAR and kobuki base.

Host source folders are bind-mounted with paths relative to the compose file, so
the workspace can live anywhere. `build/` and `install/` are named volumes
(`nav_build`, `nav_install`) that persist across restarts — use
`docker compose down -v` to force a clean rebuild.

The container does **not** build on startup. `entrypoint.sh` only sources
`/opt/ros/humble/setup.bash` and, if present, `install/setup.bash`, then idles.
Build it yourself once you are inside:

```bash
cd /root/skuba_ws
colcon build --packages-select skuba_navigation
source install/setup.bash
```

## Build (native)

```bash
cd ~/skuba_ws
colcon build --packages-select skuba_navigation
source install/setup.bash
```

Note that `maps/` and `params/` are installed into `share/`, so a `colcon build`
is required after adding a map or editing a params file — editing the source
tree alone has no effect at runtime.

## TF tree

`odom -> base_footprint` comes from the base driver; everything below it comes
from the `skuba_description` URDF. `map -> odom` comes from AMCL in
both task launches, or from SLAM Toolbox while mapping
(`nav2_slam_bringup_launch.py`).

```
map -> odom -> base_footprint -> base_link -> laser
                                           -> arm_base_link -> link-0 ... link-4 -> gripper
```

## Troubleshooting

RPLidar permission errors:

```bash
source ~/skuba_ws/src/rplidar_ros/scripts/create_udev_rules.sh
```

## Maintainers

- lukchin
- NapatzZ
