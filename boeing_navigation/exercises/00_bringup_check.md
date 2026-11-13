# Exercise 0 - Bring-up check

**Goal:** prove that the robot, the lidar and the TF tree are healthy *before* you
touch SLAM or Nav2. Most "SLAM is bad" problems are really sensor / TF problems.

## 0.1 Check the devices (host or container)

```bash
ls -l /dev/kobuki /dev/rplidar
```

Both symlinks must exist. If not, re-read the udev section of the top-level
`README.md`, reload the rules and replug the USB cables.

## 0.2 Start everything

```bash
# T1
ros2 launch boeing_bringup bringup.launch.py
```

`bringup.launch.py` starts `robot_state_publisher`, `kobuki_node` and `sllidar_node`.
The lidar motor should spin up and the Kobuki should beep.

> Different USB ports? Override them:
> `ros2 launch boeing_bringup bringup.launch.py kobuki_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0`

## 0.3 Check topics

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /odom --once
```

Expected: `/scan` and `/odom` publish at a steady rate, `/cmd_vel` exists.

## 0.4 Check the TF tree

```bash
ros2 run tf2_tools view_frames      # writes frames.pdf in the current folder
ros2 run tf2_ros tf2_echo odom laser
```

Expected tree:

```
odom -> base_link -> lidar_link -> laser
```

`odom -> base_link` comes from `kobuki_node`; the rest comes from `robot_state_publisher`
reading `boeing_description/urdf/boeing.urdf`. There is no `map` frame yet - that is
SLAM's / AMCL's job later.

## 0.5 Look at it in RViz

```bash
# T3
ros2 launch nav2_bringup rviz_launch.py
```

Set **Fixed Frame = `odom`**, add **RobotModel** and **LaserScan** (`/scan`, set the
size to ~0.03 and the reliability to *Best Effort* if nothing shows).

Checks:

1. The robot model sits flat on the grid (not tilted).
2. **Scan orientation:** put a box in front of the robot. The scan points must appear
   *in front* of the robot model. If they appear on the wrong side or rotated, the lidar
   yaw is wrong: edit the yaw of `laser_joint` in `boeing_description/urdf/boeing.urdf`
   (or the `inverted` flag in `sllidar_a3.yaml`) and re-launch.
3. **Scan is level:** walls appear as horizontal lines, not a vertical fan.

## 0.6 Drive it (carefully)

```bash
# T4
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive forward ~1 m, then rotate 360 degrees. In RViz (fixed frame `odom`) the scan of the
walls should stay in the same place while the robot moves. If the walls smear or "swim",
odometry/IMU is poor - note it, because SLAM will have to cope with it.

> Only one thing may publish `/cmd_vel` at a time. Close teleop before sending Nav2 goals.

## Questions

1. Which node publishes `odom -> base_link`? Which publishes `base_link -> laser`?
2. Why does the URDF have both `lidar_link` and `laser`?
3. What is the rate of `/scan`? What happens to SLAM if it is much lower?
4. Compare the commanded 360-degree turn with what the odometry reports. How big is the error?

## Deliverables

- Screenshot of RViz showing the robot model and the scan aligned with walls.
- `frames.pdf` of the TF tree.
- Answers to the questions above.
