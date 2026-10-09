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
loop closure on, consuming `/scan`), nav2's `navigation_launch.py` (with
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
| `nav_to_pose` | Hosts most navigation services (see below) |
| `save_location` | Writes the current pose into `share/saved_pose.yaml` |
| `door_detector` | Reports whether the door ahead is open |
| `costmap_reconfig_server` | Applies costmap presets from `params/costmap_dynamic_config.yaml` at runtime |
| `following_server` | Runs `behavior_trees/follow_point.xml` to follow a tracked person |
| `emergency_stop` | Publishes the twist_mux e-stop lock |
| `filter_lidar` | Republishes `/scan` as `/scan_filtered` with an optional angular mask |
| `map_save_cli` | Interactive prompt to serialize the SLAM Toolbox map to disk |

`cmd_vel` arbitration is handled by `twist_mux` (`params/twist_mux.yaml`): joy,
keyboard, follow and nav sources are muxed into the single `/cmd_vel` the base
drives, with `emergency_stop` holding the lock topic.

## Services

All under the `/skuba/navigation/` namespace unless noted.

From `nav_to_pose`:

| Service | Type | Notes |
| --- | --- | --- |
| `nav_to_name` | `NavToName` | go to a saved named position |
| `nav_to_position` | `NavToPose` | go to an explicit `PoseStamped` |
| `nav_to_tf` | `Nav2ToTF` | go to a standoff pose near a TF frame, via Nav2's planner (supports `approach_side`) |
| `nudge_to_tf` | `NavToTF` | move towards a TF frame via raw `cmd_vel`, no Nav2 planning |
| `rotate` | `Rotate` | in-place PD-controlled rotation |
| `move_forward` | `MoveWheel` | straight-line forward/backward move via raw `cmd_vel` |
| `cancel_goal` | `std_srvs/Trigger` | cancel the active Nav2 goal |
| `clear_costmap` | `std_srvs/Trigger` | clear local + global costmaps |
| `save_pose` | `SaveLocationCommand` | save the current pose under a name |
| `reload_yaml` | `std_srvs/Trigger` | reload saved positions from `share/saved_pose.yaml` |

From the other nodes:

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
namespace prefix, unaffected by this package's rename).

## Docker

Compose lives in `docker/`, not the package root:

```bash
cd docker
docker compose up -d
docker exec -it skuba-nav bash
```

Runs `napatku/skuba-navigation:latest` (built from `docker/dockerfile`, based on
`osrf/ros:humble-desktop-full`) with `--network host`, `ROS_DOMAIN_ID=67`,
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
