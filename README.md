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
loop closure on, consuming `/scan_slam`), nav2's `navigation_launch.py` (with
`params/nav2_params.yaml` as `params_file`), RViz, and
`person_cluster_filter` — which strips person-sized LiDAR clusters from `/scan`
and republishes the result as `/scan_slam`, so people walking past are not
baked into the occupancy grid. It also brings up `filter_lidar`,
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
| `person_cluster_filter` | Strips person-sized LiDAR clusters before SLAM (mapping only) |
| `map_save_cli` | Interactive prompt to serialize the SLAM Toolbox map to disk |

`cmd_vel` arbitration is handled by `twist_mux` (`params/twist_mux.yaml`): joy,
keyboard, follow and nav sources are muxed into the single `/cmd_vel` the base
drives, with `emergency_stop` holding the lock topic.

## navigation_server

Every way of moving the base is an **action** under `/skuba/navigation/`, so a
caller (YASMIN today, py_trees_ros later) gets feedback, can cancel its own goal,
and learns why a goal failed. Only instant operations remain services.

| Action | Type | Backend | Replaces (old `nav_to_pose` service) |
| --- | --- | --- | --- |
| `go_to_pose` | `GoToPose` | Nav2 | `nav_to_position` |
| `go_to_location` | `GoToLocation` | Nav2, saved location | `nav_to_name` |
| `go_to_frame` | `GoToFrame` | Nav2 + heading correction, standoff next to a TF (`approach_side`) | `nav_to_tf` |
| `rotate` | `RotateInPlace` | PD controller on `cmd_vel_nav_manual` | `rotate` |
| `drive_straight` | `DriveStraight` | closed loop on `cmd_vel_nav_manual` | `move_forward` |
| `nudge_to_frame` | `NudgeToFrame` | turn / drive / face on `cmd_vel_nav_manual` | `nudge_to_tf` |

| Service | Type | Notes |
| --- | --- | --- |
| `cancel_goal` | `std_srvs/Trigger` | stop whichever action goal is active (it ends ABORTED, `error.code = CANCELED`) |
| `clear_costmaps` | `std_srvs/Trigger` | clear global + local costmaps (was `clear_costmap`) |
| `save_location` | `SaveLocationCommand` | save the current pose under a name (was `save_pose`) |
| `reload_locations` | `std_srvs/Trigger` | re-read the locations file (was `reload_yaml`; reads also auto-reload on change) |

### Contract (what a behavior tree can rely on)

* **Status is authoritative:** `SUCCEEDED` ⇔ `result.error.code == NONE`
  (`skuba_interfaces/msg/NavigationError`). `py_trees_ros.actions.ActionClient`
  maps that straight to SUCCESS / FAILURE; `error.code` says how to recover.
* **One goal at a time, newest wins.** A new goal on *any* of the six actions
  preempts the running one (`ABORTED`, `PREEMPTED`) and starts only once the
  old one has stopped the base.
* **Cancel is cheap and final.** Cancelling (e.g. a BT branch being interrupted)
  cancels the Nav2 goal / stops the base within one control tick, ends
  `CANCELED`, and is never retried.
* **Bad goals are rejected at goal time** (unknown location, unknown
  `approach_side`, NaN, zero quaternion), so the tree fails on the same tick.
* **`timeout` is a total deadline** across Nav2 retries; `<= 0` uses the node default.
* **Retries** happen only when Nav2 `ABORTED` (costmaps cleared first), up to
  `navigation.max_retries`.
* **Feedback** is capped at `navigation.feedback_rate` (5 Hz); phase changes are always sent.

```python
# py_trees_ros
from py_trees_ros.actions import ActionClient
from skuba_interfaces.action import GoToLocation

go_kitchen = ActionClient(
    name='GoKitchen', action_type=GoToLocation,
    action_name='/skuba/navigation/go_to_location',
    action_goal=GoToLocation.Goal(location='kitchen', timeout=60.0))
```

```bash
ros2 action send_goal --feedback /skuba/navigation/go_to_frame skuba_interfaces/action/GoToFrame \
  "{frame: cup_0, stop_distance: 0.5, approach_side: FRONT, object_radius: 0.1}"
ros2 service call /skuba/navigation/cancel_goal std_srvs/srv/Trigger
```

### Layout

```
nodes/navigation_server_node.py   ROS glue: params, the action table, services
adapters/                         rclpy implementations of the core ports
  nav2_client.py                  NavigateToPose / ComputePathToPose / ClearEntireCostmap clients
  tf_pose_source.py, cmd_vel_base.py, conversions.py, parameters.py
core/navigation/                  no rclpy - unit-tested with fakes (tests/test_navigation_*.py)
  tasks.py                        one function per action - start reading here
  navigation_job.py               Nav2 pipeline: clear, plan-check, send, retry, cancel, deadline
  motion.py                       closed-loop rotate / drive-straight
  standoff.py, location_store.py, validation.py, cancellation.py, retry_policy.py, ...
```

Adding an action = one `.action` in skuba_interfaces, one function in
`tasks.py`, one row in `_action_specs()`. Tunables live in
`core/navigation/config.py`; each field is a ROS parameter
(`params/navigation_server.yaml`, kept in sync by a test).

Saved locations: the `locations_file` parameter, or by default
`<ws>/src/skuba_navigation/share/saved_pose.yaml` (also `skuba-navigation/`),
falling back to the installed share directory.

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
