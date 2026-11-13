# Exercise 2 - Navigating on a saved map

**Goal:** localise the robot on the map from Exercise 1 with AMCL and drive to goals
with Nav2.

Background: with a saved map, `map_server` publishes `/map`, **AMCL** estimates
`map -> odom` by matching the scan against the map, and Nav2 plans
(`planner_server`) and follows the path (`controller_server`). Commands go out on
`/cmd_vel` (through the velocity smoother) to `kobuki_node`.

## 2.1 Start

Stop SLAM (T2) first - only one node may publish `map -> odom`.

```bash
# T1 (keep running)
ros2 launch boeing_bringup bringup.launch.py

# T2
ros2 launch boeing_navigation navigation.launch.py \
  map:=/ros_ws/src/boeing_navigation/maps/lab.yaml
```

`navigation.launch.py` starts Nav2 (`bringup_launch.py`: map server, AMCL, planner,
controller, BT navigator, ...) with `config/nav2_params.yaml`, plus RViz
(`rviz:=false` to skip it if RViz runs elsewhere).

## 2.2 Give the initial pose

AMCL does not know where the robot is. Place the robot at a recognisable spot on the map,
then in RViz click **2D Pose Estimate** and drag an arrow at that position/heading.

Verify:

- The scan overlays the walls of the map.
- Particle cloud (`/particle_cloud`) shrinks to a tight blob after you drive a little.
- If it does not, set the pose again, or drive around (`teleop`) to help convergence.

## 2.3 Send goals

1. Click **Nav2 Goal** in RViz and set a goal ~2 m away.
2. Watch the green global path and the local plan; the robot should drive there.
3. Try goals: across the room, around a corner, through a doorway, and a goal inside a wall
   (what happens?).
4. Put an obstacle (a box, a chair) on the path *after* the robot starts moving.
   Watch the local costmap and see how the robot reacts.

You can also send goals from the terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

In RViz enable these displays and look at each one: **Global Costmap**, **Local Costmap**,
**Global Plan**, **Local Plan**, **Footprint**.

## 2.4 Things to observe

- Does localisation stay locked while driving? When does it fail (long corridors, people,
  fast rotation)?
- How close to walls does the robot drive? Does it get stuck in doorways?
- How does the robot recover when blocked? (Behaviour tree: clear costmap, spin, back up, wait.)

## Questions

1. Which node publishes `map -> odom` now? Which one did in Exercise 1?
2. Why is the *global* costmap in the `map` frame but the *local* costmap in the `odom` frame?
3. What topics does the controller subscribe to and publish? Draw the `/cmd_vel` path:
   controller -> `cmd_vel_nav` -> velocity smoother -> `/cmd_vel` -> `kobuki_node`.
   (Hint: `ros2 topic info -v`, `rqt_graph`.)
4. What happens if you start navigation without setting the initial pose?

## Deliverables

- Screenshot of a successful navigation (path + costmaps visible).
- List of 3 failures you observed, with what you think caused each (to be fixed in Exercise 3).
