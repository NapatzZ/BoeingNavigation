# Exercise 3 - Tuning after mapping

**Goal:** make navigation on your saved map smooth, safe and reliable by tuning
`boeing_navigation/config/nav2_params.yaml` *methodically*.

## 3.0 Rules of tuning

1. **Baseline first.** Fix a test course (e.g. A -> B -> C -> A, including one doorway/narrow gap)
   and run it 3 times with the unchanged config.
2. **One change at a time.** Change one parameter (or one tightly related group), re-test.
3. **Write it down.** Use the results table at the bottom.
4. **Same conditions.** Same start pose, same lighting/obstacles, same battery level if possible.
5. Revert changes that don't help. `git diff` is your friend.

Metrics per run: reached goal (Y/N), time, number of recoveries (spin/back-up/wait),
closest distance to a wall, smoothness (any jerks/oscillation?).

## 3.1 Live tuning

Many Nav2 parameters can be changed while running, which is much faster than restarting:

```bash
ros2 param list /controller_server | grep FollowPath
ros2 param get  /controller_server FollowPath.max_vel_x
ros2 param set /controller_server FollowPath.max_vel_x 0.18
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.45
```

If a parameter does not take effect, it is not dynamic - edit the YAML and restart T2.
**When you find good values, write them into `nav2_params.yaml`** (live changes are lost on restart).

## 3.2 Tuning topics

Work through these in order. Each has *symptom -> parameter -> what to try*.

### A. Robot footprint and inflation (safety margin)

Symptoms: scrapes walls; or refuses to enter doorways / says "no path" when the gap is big enough.

| Parameter (both costmaps) | Now | Notes |
|---|---|---|
| `robot_radius` | 0.18 | Measure the real robot. Too small = collisions, too large = can't pass doors |
| `inflation_layer.inflation_radius` | 0.55 | Distance around obstacles that is penalised |
| `inflation_layer.cost_scaling_factor` | 3.0 | Higher = cost drops faster = robot hugs obstacles; lower = keeps to the middle |

Try: `inflation_radius` 0.35 / 0.55 / 0.8 and `cost_scaling_factor` 2 / 3 / 6.
Rule of thumb: inflation radius must be bigger than the robot radius and smaller than half
the narrowest gap you want to pass **plus** the radius. Keep **local and global in sync**.

### B. Speed and acceleration

Symptoms: robot jerks, overshoots goals, or is unnecessarily slow.

| Parameter | Now | Notes |
|---|---|---|
| `FollowPath.max_vel_x`, `max_speed_xy` | 0.26 | Top linear speed |
| `FollowPath.max_vel_theta` | 1.0 | Top rotation speed |
| `FollowPath.acc_lim_x`, `decel_lim_x` | 2.5 / -2.5 | **Compare with the real base** |
| `velocity_smoother.max_velocity / max_accel` | see file | Must be consistent with the controller |

Important: `kobuki_node.yaml` has its own acceleration limiter
(`linear_acceleration_limit: 0.2`, `linear_deceleration_limit: -0.3`). If the controller
assumes 2.5 m/s^2 but the base can only do 0.2, the real motion lags behind the plan and
tracking gets worse. Experiment: set `acc_lim_x` and the smoother limits close to what the base
really does, and compare. Also try `acceleration_limiter: false` in `kobuki_node.yaml`
(carefully, and only at low speed) and see what changes.

### C. DWB critics (path-following behaviour)

Symptoms: cuts corners, wobbles around the path, ignores the path, hesitates near obstacles.

```
PathAlign.scale 32   PathDist.scale 32   GoalAlign.scale 24
GoalDist.scale 24    BaseObstacle.scale 0.02   RotateToGoal.scale 32
```

- Increase `PathDist` / `PathAlign` -> stick closer to the global path.
- Increase `GoalDist` / `GoalAlign` -> go to the goal more aggressively.
- Increase `BaseObstacle` -> stay farther from obstacles (use with inflation).
- `sim_time` (1.7) -> longer sees farther ahead (smoother, but cuts corners / slower in tight spaces).
- `vx_samples`, `vtheta_samples` -> more samples = better choice, more CPU.

### D. Goal tolerances

Symptoms: oscillates near the goal, stops too early, never "arrives".

`general_goal_checker.xy_goal_tolerance` and `yaw_goal_tolerance` (0.25 each) and
`FollowPath.xy_goal_tolerance`. Try 0.10 / 0.15 and note if it starts to oscillate.

### E. Sensor ranges in the costmaps

`raytrace_max_range` (3.0), `obstacle_max_range` (2.5) in both costmaps.

- Too small: robot sees obstacles late.
- Too large: noise and ghost obstacles. The A3 sees much farther than the costmap uses -
  decide how far you really need.
- `clearing: True` removes obstacles when the beam passes through; check that moved
  objects disappear from the costmap.

Also try the local costmap `width/height` (3 m) and `update_frequency` (5 Hz).

### F. Global planner

Symptoms: path hugs corners, odd wiggles, long detours, or fails to find a path.

`GridBased.use_astar` (false -> try true), `tolerance` (0.5), `allow_unknown` (true).
Also try the smoother (`smoother_server`) parameters if paths look jagged.

### G. Localisation (AMCL)

Symptoms: pose jumps, drifts in corridors, slow to converge.

| Parameter | Now | Try |
|---|---|---|
| `max_particles` / `min_particles` | 2000 / 500 | fewer = faster, more = more robust |
| `update_min_d`, `update_min_a` | 0.25 / 0.2 | smaller = more frequent updates |
| `alpha1..alpha5` | 0.2 | odometry noise model; larger = trust odometry less |
| `laser_max_range` | 100 | set to a realistic value for the room |
| `sigma_hit`, `z_hit`, `z_rand` | 0.2 / 0.5 / 0.5 | scan model |
| `max_beams` | 60 | more = more accurate, more CPU |

### H. Recovery behaviours

Observe what the robot does when stuck (spin, back up, wait, clear costmap).
Look at the behaviour tree in `bt_navigator` (the default is used) and `behavior_server` params.

## 3.3 Debugging tools

- RViz displays: Global/Local Costmap, Plans, Footprint, `/particle_cloud`, DWB trajectory markers.
- `ros2 topic echo /cmd_vel` - what the base actually receives.
- `ros2 topic hz /cmd_vel`, `/odom`, `/scan` - frame rates.
- `ros2 run tf2_ros tf2_echo map base_link` - localisation output.
- `ros2 lifecycle get /controller_server` - are Nav2 nodes active?
- `ros2 bag record /scan /odom /tf /tf_static /cmd_vel /plan /local_plan` - record a bad
  run and replay/inspect it later.
- Nav2 logs in the terminal (T2): warnings like "Failed to make progress",
  "Control loop missed its desired rate", "Costmap timed out".

## 3.4 Results table (copy and fill in)

| # | Change (file: parameter old -> new) | Reached goal? | Time (s) | Recoveries | Min wall dist | Notes |
|---|---|---|---|---|---|---|
| 0 | baseline | | | | | |
| 1 | | | | | | |
| 2 | | | | | | |

## Questions

1. What is the trade-off controlled by `inflation_radius` and `cost_scaling_factor`?
2. Why must the controller's `acc_lim_*` be consistent with the base's own limiter?
3. What does each of `PathAlign`, `PathDist`, `GoalAlign`, `GoalDist` reward?
4. You see the robot oscillating near the goal. Name three parameters you would check.
5. The pose estimate slowly drifts along a long corridor. Why, and what could help?

## Deliverables

- Your final `nav2_params.yaml` (commit it) with comments on what changed and why.
- The completed results table with at least 8 experiments.
- A short before/after comparison on the test course (video or screenshots).
