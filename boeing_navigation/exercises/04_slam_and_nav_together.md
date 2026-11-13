# Exercise 4 - Online SLAM + Nav2 together

**Goal:** navigate in a place that has **no saved map**, while slam_toolbox builds the map
at the same time. This is how you handle new/changing environments.

Differences from Exercise 2:

| | Saved-map navigation (Ex. 2) | SLAM + Nav2 (this exercise) |
|---|---|---|
| `/map` | `map_server` (static) | slam_toolbox (growing, updating) |
| `map -> odom` | AMCL | slam_toolbox |
| Needs initial pose? | Yes | No (starts at the origin) |
| Map changes while driving? | No | Yes, including jumps at loop closure |

So **do not run `map_server` or AMCL** here - they would fight slam_toolbox. We only need
Nav2's *navigation* part (planner, controller, BT navigator, ...).

## 4.1 Start

```bash
# T1
ros2 launch boeing_bringup bringup.launch.py

# T2 - SLAM (publishes /map and map -> odom)
ros2 launch boeing_navigation slam.launch.py

# T3 - Nav2 navigation only (no map_server, no AMCL)
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/ros_ws/src/boeing_navigation/config/nav2_params.yaml

# T4
ros2 launch nav2_bringup rviz_launch.py
```

Check that everything is active:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
```

## 4.2 Explore with goals

1. Drive a metre or two with teleop first so slam_toolbox has some map, then **close teleop**.
2. In RViz click **Nav2 Goal** at the edge of the known (white) area.
3. Repeat, pushing the goal into new places: the "frontier" between free and unknown.
4. Why does the planner accept goals in unknown space? Look at `allow_unknown: true` in
   `planner_server` and `track_unknown_space: true` in the global costmap.

Do a full exploration of a room by sending goals only (no teleop).

## 4.3 What to watch for

- **Map jumps:** after a loop closure the map (and your plan) shifts. The global plan may become
  invalid and Nav2 will replan; sometimes it aborts the goal.
- **Costmap vs map update rate:** slam_toolbox updates the map every `map_update_interval`
  (5 s). The global costmap's static layer follows. New walls appear with a delay; the
  scan-based obstacle layer fills the gap.
- **Speed matters more than before:** driving fast makes both SLAM and DWB worse.
  Lower `max_vel_x` / `max_vel_theta` while exploring.
- **Unknown cells:** the planner may route through unknown space and the robot only discovers
  the wall when it gets there.

## 4.4 Experiments

| Experiment | What to change | What to observe |
|---|---|---|
| A. Update interval | `map_update_interval` 5.0 -> 1.0 / 10.0 | Map freshness vs CPU load |
| B. Planner over unknown | `allow_unknown` true -> false | Can it still explore with goals in unknown space? |
| C. Speed | `max_vel_x` 0.26 -> 0.15 / 0.4 | Map quality, localisation, tracking |
| D. Loop closure | drive a loop; compare with `do_loop_closing: false` | Map consistency, plan stability |
| E. Dynamic obstacle | walk in front of the robot | Do people leave "ghost" walls in the map? |

## 4.5 Save the result and go back to saved-map mode

When the room is explored:

```bash
ros2 run nav2_map_server map_saver_cli -f /ros_ws/src/boeing_navigation/maps/explored
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/ros_ws/src/boeing_navigation/maps/explored_graph'}"
```

Stop everything, then repeat Exercise 2 with `explored.yaml`. Compare the experience
(localisation quality, navigation) against the map from Exercise 1.

## Stretch goals

- **Frontier exploration:** write (or install) a node that picks the nearest frontier and
  calls the `navigate_to_pose` action in a loop, so the robot explores autonomously.
- **One launch file:** create `boeing_navigation/launch/slam_nav.launch.py` that includes
  `slam.launch.py` and Nav2's `navigation_launch.py` (and optionally RViz) so T2 + T3 + T4 become one command.
  (New files need `colcon build`.)
- **Localisation mode:** continue from the serialised pose graph with `mode: localization` in
  `slam_toolbox.yaml` instead of AMCL, and compare the two.
- **Record and replay:** `ros2 bag record` the run, then replay with `--clock` and
  `use_sim_time:=true` to tune SLAM parameters offline.

## Questions

1. Why must `map_server` and AMCL **not** run at the same time as slam_toolbox?
2. What happens to Nav2 when the map shifts by 20 cm at a loop closure while the robot is
   following a plan? Which components react?
3. When would you choose online SLAM + Nav2 over "map once, then AMCL"? List pros and cons.
4. How do the costmap `static_layer` and `obstacle_layer` complement each other here?

## Deliverables

- A map produced purely by sending Nav2 goals (screenshot + saved files).
- Notes on at least 3 problems that appeared that did not appear in Exercise 2, and how you handled them.
- Your answers to the questions above.
