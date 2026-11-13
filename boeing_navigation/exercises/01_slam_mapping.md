# Exercise 1 - Mapping with SLAM

**Goal:** build a clean 2D occupancy-grid map of a room/corridor with slam_toolbox
and save it.

Background: SLAM estimates the robot pose *and* the map at the same time. slam_toolbox
publishes the `map -> odom` transform and the `/map` topic; odometry gives `odom -> base_link`.

## 1.1 Start

```bash
# T1
ros2 launch boeing_bringup bringup.launch.py
# T2
ros2 launch boeing_navigation slam.launch.py
# T3
ros2 launch nav2_bringup rviz_launch.py      # Fixed Frame: map, add the Map display on /map
# T4
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`slam.launch.py` runs slam_toolbox *online async* with
`boeing_navigation/config/slam_toolbox.yaml`.

Check the TF tree again: it must now be `map -> odom -> base_link -> lidar_link -> laser`.

## 1.2 Map a space

Pick a room or corridor with walls and a few features (not a long featureless hallway).

Good mapping technique:

- Drive **slowly** (about 0.1-0.2 m/s) and turn slowly. Fast rotation is the #1 cause of
  smeared maps.
- Keep walls within lidar range and avoid people walking through the scan.
- Drive along the walls first, then through the middle.
- **Close the loop:** come back to where you started and drive over the same area again so
  slam_toolbox can do loop closure. Watch the map "snap" into place.
- Don't stop to spin on the spot repeatedly; it builds up odometry error.

## 1.3 Save the map

When the map looks complete:

```bash
# T4  (use an absolute path; maps/ in src is not re-installed automatically)
ros2 run nav2_map_server map_saver_cli -f /ros_ws/src/boeing_navigation/maps/lab
```

This writes `lab.yaml` + `lab.pgm`. Open the `.pgm` in an image viewer: white = free,
black = occupied, grey = unknown.

Also save the **pose graph**, which lets you continue mapping later:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/ros_ws/src/boeing_navigation/maps/lab_graph'}"
```

> If the container is removed, files inside it are lost. Copy them out
> (`docker cp <container>:/ros_ws/src/boeing_navigation/maps ./maps`) or mount the folder.
> Commit good maps to git.

## 1.4 Experiments (change one thing at a time)

Edit `boeing_navigation/config/slam_toolbox.yaml`, restart T2, and re-map the *same*
route. Compare screenshots.

| Experiment | Parameter | Expected effect |
|------------|-----------|-----------------|
| A. No loop closing | `do_loop_closing: false` | Map drifts / doubled walls when you return |
| B. Update more/less often | `minimum_travel_distance`, `minimum_travel_heading` | Smaller = more scans, denser graph, more CPU |
| C. Range limit | `max_laser_range` | Smaller = less far-away noise; too small = fewer features |
| D. Resolution | `resolution` (0.05 -> 0.03 / 0.1) | Detail vs map size and CPU |
| E. Speed | drive 2x faster | See how the map degrades |

## Questions

1. What is the difference between `odom` and `map` frames? Why does `map -> odom` jump
   at a loop closure?
2. What does `use_scan_matching` do, and what would happen with `false`?
3. Why is a long, featureless corridor hard for 2D lidar SLAM?
4. How does `resolution` affect the saved `.pgm` size and later planning?

## Deliverables

- `maps/<name>.yaml` + `.pgm` (and the pose graph files).
- A before/after screenshot for at least two experiments (A-E).
- Short notes: what made the map worse, what fixed it.
