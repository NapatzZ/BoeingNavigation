# navigation_server

`navigation_server` is the single node that moves the SKUBA base. Every
motion is a ROS 2 **action** (feedback, cancel, typed failure reason); only
instant operations are services. It replaces the old `nav_to_pose` service
node.

```bash
ros2 run skuba_navigation navigation_server --ros-args \
  --params-file $(ros2 pkg prefix skuba_navigation)/share/skuba_navigation/params/navigation_server.yaml
```

Both task launch files (`cheesecake_nav.launch.py`,
`cheesecake_resturant.launch.py`) start it with that params file.

- [What changed (migration)](#what-changed-migration)
- [Actions](#actions)
- [Services](#services)
- [Error codes](#error-codes)
- [Behaviour contract](#behaviour-contract)
- [Cancelling goals](#cancelling-goals)
- [Using it from a behavior tree / state machine](#using-it-from-a-behavior-tree--state-machine)
- [Parameters](#parameters)
- [Code layout](#code-layout)
- [Adding a new action](#adding-a-new-action)
- [Testing](#testing)

---

## What changed (migration)

All endpoints of this package moved from `/skuba/navigation/` to
**`/skuba/nav/`**, and the blocking navigation services became actions.
Interface types come from `skuba_interfaces` (branch
`feat/navigation-actions`).

| Old (`nav_to_pose`, service) | New (`navigation_server`) | Kind |
| --- | --- | --- |
| `/skuba/navigation/nav_to_position` (`NavToPose`) | `/skuba/nav/go_to_pose` (`GoToPose`) | action |
| `/skuba/navigation/nav_to_name` (`NavToName`) | `/skuba/nav/go_to_location` (`GoToLocation`) | action |
| `/skuba/navigation/nav_to_tf` (`Nav2ToTF`) | `/skuba/nav/go_to_frame` (`GoToFrame`) | action |
| `/skuba/navigation/rotate` (`Rotate`) | `/skuba/nav/rotate` (`RotateInPlace`) | action |
| `/skuba/navigation/move_forward` (`MoveWheel`) | `/skuba/nav/drive_straight` (`DriveStraight`) | action |
| `/skuba/navigation/nudge_to_tf` (`NavToTF`) | `/skuba/nav/nudge_to_frame` (`NudgeToFrame`) | action |
| `/skuba/navigation/cancel_goal` (`Trigger`) | *removed* — use action cancel ([below](#cancelling-goals)) | — |
| `/skuba/navigation/clear_costmap` (`Trigger`) | `/skuba/nav/clear_costmaps` (`Trigger`) | service |
| `/skuba/navigation/save_pose` (`SaveLocationCommand`) | `/skuba/nav/save_location` (`SaveLocationCommand`) | service |
| `/skuba/navigation/reload_yaml` (`Trigger`) | `/skuba/nav/reload_locations` (`Trigger`) | service |

The other nodes in the package only changed prefix, e.g.
`/skuba/navigation/filter_lidar` → `/skuba/nav/filter_lidar`,
`/skuba/navigation/person_pose` → `/skuba/nav/person_pose` (update the vision
follower that publishes it).

Old field → new field, for callers being ported:

| Old request field | New goal field |
| --- | --- |
| `NavToName.pose` | `GoToLocation.location` |
| `NavToPose.pose` | `GoToPose.pose` |
| `Nav2ToTF.object_tf_name` / `stop_distance` / `approach_side` / `object_radius` / `timeout` (int) | `GoToFrame.frame` / `stop_distance` / `approach_side` / `object_radius` / `timeout` (float) |
| `Rotate.degrees` / `tolerance_degrees` | `RotateInPlace.angle_deg` / `tolerance_deg` |
| `MoveWheel.distance` | `DriveStraight.distance` |
| `NavToTF.object_tf_name` / `stop_distance` | `NudgeToFrame.frame` / `stop_distance` |
| response `bool success` | goal status + `result.error` (`code`, `message`) |

Behaviour differences worth knowing:

- `timeout` is now a **total** deadline across Nav2 retries (it used to be per
  attempt, so `timeout=30` could take 90 s). `<= 0` means node default.
- A cancel is final; the old node retried a cancelled goal up to twice.
- Motions that cannot verify the final pose now **fail** instead of reporting
  success; a TF dropout longer than `motion.tf_grace` stops the base and fails
  with `TF_LOOKUP_FAILED` (no more driving blind / time-based fallback).
- `rotate` handles any angle (e.g. 270°) and turns the requested way.
- `nudge_to_frame` turns with the same PD controller as `rotate`.

---

## Actions

All actions share these rules: see [Behaviour contract](#behaviour-contract).
The full `.action` definitions (with comments) are in
`skuba_interfaces/action/`.

### `/skuba/nav/go_to_pose` — `GoToPose` (Nav2)

| Goal | |
| --- | --- |
| `geometry_msgs/PoseStamped pose` | target; empty `frame_id` = `map` |
| `float32 timeout` | s, total; `<= 0` = `navigation.default_timeout` |
| `string behavior_tree` | BT XML path for this goal; empty = bt_navigator default |

Result: `error`, `final_pose`, `attempts`, `elapsed`.
Feedback: `NavigationProgress progress` (see below).

### `/skuba/nav/go_to_location` — `GoToLocation` (Nav2)

Goal: `string location`, `float32 timeout`, `string behavior_tree`.
The name is looked up in the locations file (see
[`save_location`](#services)); an unknown name is **rejected**.
Result/feedback as `GoToPose`.

### `/skuba/nav/go_to_frame` — `GoToFrame` (Nav2 + heading correction)

Drive to a standoff pose next to a TF frame, turn to face it, verify.

| Goal | |
| --- | --- |
| `string frame` | target TF frame, e.g. `cup_0` |
| `float32 stop_distance` | m between the robot and the target |
| `string approach_side` | `FRONT` `FRONT_LEFT` `LEFT` `BACK_LEFT` `BACK` `BACK_RIGHT` `RIGHT` `FRONT_RIGHT`, or empty/`AUTO` |
| `float32 object_radius` | m added to `stop_distance` (named sides only) |
| `float32 timeout`, `string behavior_tree` | as above |

- Sides are **map-relative** (`FRONT` = map +X, `LEFT` = map +Y), because
  object TFs carry no usable heading. `AUTO` stops short along the line the
  robot is already on.
- A named side the planner cannot reach fails with `NO_VALID_PATH`; it never
  falls back to another side. An unknown side is rejected.
- The final pose check is authoritative: if Nav2 aborts or times out but the
  robot is in fact parked correctly, the goal **succeeds**.

Result: `error`, `final_pose`, `final_distance`, `attempts`, `elapsed`.
Feedback: `progress` (phases go PLANNING → NAVIGATING → ALIGNING → VERIFYING).

### `NavigationProgress` (feedback of the three Nav2 actions)

| Field | |
| --- | --- |
| `phase` | `PHASE_RESOLVING` 0, `PLANNING` 1, `NAVIGATING` 2, `RETRYING` 3, `ALIGNING` 4, `VERIFYING` 5 |
| `attempt` | 1-based Nav2 attempt |
| `target_pose`, `current_pose` | goal sent to Nav2 / robot pose from Nav2 |
| `distance_remaining`, `estimated_time_remaining`, `number_of_recoveries` | from Nav2 feedback |
| `elapsed` | s since the goal started |

### `/skuba/nav/rotate` — `RotateInPlace` (cmd_vel)

Goal: `angle_deg` (relative, + = CCW, any magnitude), `tolerance_deg` (`<= 0` =
`motion.rotate_default_tolerance_deg`, 10°), `timeout`.
Result: `error`, `achieved_angle_deg`, `elapsed`.
Feedback: `remaining_angle_deg`, `elapsed`.

### `/skuba/nav/drive_straight` — `DriveStraight` (cmd_vel)

Goal: `distance` (m, negative = backward), `speed` (m/s, `<= 0` = 0.2),
`timeout` (`<= 0` = `|distance| / speed + 5 s`).
Result: `error`, `achieved_distance` (signed, measured along the start
heading), `elapsed`. Feedback: `remaining_distance`, `elapsed`.

### `/skuba/nav/nudge_to_frame` — `NudgeToFrame` (cmd_vel)

Short-range approach without Nav2: turn toward `frame`, drive until
`stop_distance` away, turn to face it. For the last half metre, not travel —
**no collision checking**.
Goal: `frame`, `stop_distance`, `timeout`.
Result: `error`, `final_distance`, `elapsed`.
Feedback: `phase` (`PHASE_ALIGNING` 0, `PHASE_DRIVING` 1, `PHASE_FACING` 2),
`remaining_distance`, `elapsed`.

The three cmd_vel actions publish on `cmd_vel_nav_manual`, the manual
twist_mux lane (`params/twist_mux.yaml`), never on Nav2's `cmd_vel_nav`.

---

## Services

| Service | Type | |
| --- | --- | --- |
| `/skuba/nav/clear_costmaps` | `std_srvs/Trigger` | clear global + local costmaps |
| `/skuba/nav/save_location` | `skuba_interfaces/SaveLocationCommand` | save the current `map → base_link` pose under `location_name` |
| `/skuba/nav/reload_locations` | `std_srvs/Trigger` | force a re-read of the locations file |

Poses are tied to a map: each map has `<map>.poses.yaml` next to its yaml
(`maps/30_skuba_home/skuba_map_real.yaml` -> `skuba_map_real.poses.yaml`). The
server asks Nav2's `map_server` which map is running (its `yaml_filename`
parameter, 1 s timeout; asked again after every new `/map` or when the service
went away) and uses that map's file, in the source checkout when it has the
map. `map_server` never updates that parameter after a runtime `load_map`, so
it is checked against the map on `/map` (size, resolution, origin). A swapped
map is recognised when exactly one saved map matches; otherwise it counts as
no map loaded. A reported map that cannot be read is trusted only when no saved
map matches `/map` either. The `map` parameter names the
map yaml instead of asking. With no map loaded (the `mapping` task, Nav2 not
up, or an unclear swap) `save_location` fails and `go_to_location` ends with
`LOCATION_NOT_FOUND` ("no map loaded"); nothing falls back to another map's
poses. The `locations_file` parameter pins one file for every map.
Reads pick up external edits (the `save_location` node, the map editor)
automatically; writes merge into the file and replace it atomically.

---

## Error codes

`skuba_interfaces/msg/NavigationError` — every result carries `error.code` and
a human-readable `error.message`.

| Code | Name | Typical recovery |
| --- | --- | --- |
| 0 | `NONE` | — (status `SUCCEEDED`) |
| 1 | `UNKNOWN` | check the node log |
| 2 | `INVALID_GOAL` | fix the goal |
| 3 | `LOCATION_NOT_FOUND` | save / reload locations |
| 4 | `TF_LOOKUP_FAILED` | frame not published / localization lost |
| 5 | `NAV2_UNAVAILABLE` | Nav2 not running |
| 6 | `GOAL_REJECTED` | Nav2 refused (e.g. bad BT path) |
| 7 | `NO_VALID_PATH` | pick another side / location |
| 8 | `NAVIGATION_FAILED` | Nav2 aborted every attempt — clear costmaps, try again |
| 9 | `TIMEOUT` | deadline elapsed |
| 10 | `CANCELED` | someone cancelled |
| 11 | `PREEMPTED` | a newer goal took over |
| 12 | `VERIFICATION_FAILED` | motion ended outside tolerance |

---

## Behaviour contract

What a behavior tree (py_trees_ros) or state machine (YASMIN) can rely on:

1. **Status is authoritative.** `SUCCEEDED` ⇔ `error.code == NONE`. Anything
   else ends `ABORTED`, or `CANCELED` when the client itself cancelled.
2. **One goal at a time, newest wins.** A new goal on *any* of the six actions
   preempts the running one (it ends `ABORTED` / `PREEMPTED`). The new goal
   starts only after the old one has stopped the base, so two controllers
   never drive together.
3. **Cancel is cheap and final.** It cancels the Nav2 goal or stops the
   cmd_vel loop within one control tick, and is never retried.
4. **Bad goals are rejected at goal time** (empty/unknown location, unknown
   `approach_side`, NaN, zero quaternion, negative distances) — the reason is
   logged by the node.
5. **Retries** only when Nav2 returns `ABORTED`, after clearing costmaps, up to
   `navigation.max_retries` and within `timeout`.
6. **Feedback** is rate-limited to `navigation.feedback_rate` (5 Hz); phase
   changes are always sent.
7. **Shutdown** (Ctrl-C / SIGTERM) cancels the active goal and publishes zero
   velocity before exiting.

---

## Cancelling goals

There is no `cancel_goal` service any more — the action protocol already has one.

- **The caller cancels its own goal** — `goal_handle.cancel_goal_async()` in
  rclpy; py_trees_ros does this automatically when a running `ActionClient`
  behaviour is interrupted; YASMIN `ActionState` does it on cancel.
- **Anyone cancels everything on one action** — every action has a
  `<action>/_action/cancel_goal` service; a request with an empty goal id and
  zero stamp means "cancel all goals":

  ```bash
  ros2 service call /skuba/nav/go_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
  ```

  Because of the one-goal-at-a-time rule, only one action can be active; if
  you do not know which, call it on each of the six actions (the idle ones
  answer with an empty list).
- **Ctrl-C in `ros2 action send_goal`** cancels that goal.

---

## Using it from a behavior tree / state machine

py_trees_ros:

```python
from py_trees_ros.actions import ActionClient
from skuba_interfaces.action import GoToFrame, GoToLocation

go_to_table = ActionClient(
    name='GoToTable', action_type=GoToLocation,
    action_name='/skuba/nav/go_to_location',
    action_goal=GoToLocation.Goal(location='table', timeout=60.0))

approach_cup = ActionClient(
    name='ApproachCup', action_type=GoToFrame,
    action_name='/skuba/nav/go_to_frame',
    action_goal=GoToFrame.Goal(frame='cup_0', stop_distance=0.5,
                               approach_side='FRONT', object_radius=0.05))
```

YASMIN (sketch — `yasmin_ros.ActionState` maps the goal status to
`SUCCEED` / `ABORT` / `CANCEL` itself; the result handler runs on success):

```python
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED
from skuba_interfaces.action import GoToLocation


class GoToLocationState(ActionState):
    def __init__(self, location, timeout=60.0):
        super().__init__(GoToLocation, '/skuba/nav/go_to_location',
                         self.create_goal, result_handler=self.on_result)
        self.location, self.timeout = location, timeout

    def create_goal(self, blackboard):
        return GoToLocation.Goal(location=self.location, timeout=self.timeout)

    def on_result(self, blackboard, result):
        return SUCCEED
```

CLI:

```bash
ros2 action send_goal --feedback /skuba/nav/go_to_location skuba_interfaces/action/GoToLocation "{location: kitchen}"
ros2 action send_goal /skuba/nav/rotate skuba_interfaces/action/RotateInPlace "{angle_deg: 90.0}"
ros2 service call /skuba/nav/save_location skuba_interfaces/srv/SaveLocationCommand "{location_name: kitchen}"
```

---

## Parameters

`params/navigation_server.yaml` lists every parameter with its default.

| Parameter | Default | |
| --- | --- | --- |
| `map_frame` / `robot_frame` | `map` / `base_link` | frames for TF lookups |
| `cmd_vel_topic` | `cmd_vel_nav_manual` | twist_mux lane for the cmd_vel actions |
| `locations_file` | `''` | one poses file for every map, see [Services](#services) |
| `preempt_handover_timeout` | 5.0 s | how long a new goal waits for the old one to stop |
| `nav2.*` | Nav2 defaults | action / service names, `server_timeout`, `cancel_timeout` |
| `navigation.*` | | Nav2 goals: `default_timeout`, `max_retries`, `retry_delay`, costmap clearing, `feedback_rate`, standoff / facing tolerances |
| `motion.*` | | rotation PD gains and limits, drive speed / tolerances, nudge thresholds, `tf_grace`, `stop_repeats` |

The `navigation.*` and `motion.*` groups are generated from the dataclasses in
`core/navigation/config.py` — add a field there and it becomes a parameter; a
unit test fails if the YAML and the dataclass disagree.

---

## Code layout

```
skuba_navigation/
├── nodes/navigation_server_node.py   ROS glue only: params, one ActionSpec row per action, services
├── adapters/                         rclpy, but not Nodes — implement core/navigation/ports.py
│   ├── nav2_client.py                NavigateToPose / ComputePathToPose actions, ClearEntireCostmap
│   ├── tf_pose_source.py             robot / frame poses from tf2
│   ├── cmd_vel_base.py               Twist publisher
│   ├── conversions.py                core types <-> ROS messages
│   └── parameters.py                 dataclass -> ROS parameters
└── core/navigation/                  no rclpy import — unit-tested with fakes
    ├── tasks.py                      ONE FUNCTION PER ACTION — start reading here
    ├── navigation_job.py             Nav2 pipeline: clear → plan-check → send → retry / cancel / deadline
    ├── motion.py                     closed-loop rotate / drive-straight controllers
    ├── standoff.py                   where to park next to a frame, and verification
    ├── location_store.py             <map>.poses.yaml
    ├── validation.py                 goal-time checks
    ├── cancellation.py               CancelToken + GoalArbiter (one goal at a time)
    ├── retry_policy.py               Nav2 outcome → retry / finish
    ├── ports.py                      Protocols the adapters implement
    ├── clock.py, config.py, types.py
```

Request flow: action server → `_on_goal` (validation) →
`_execute` (acquire the arbiter, build a `TaskContext`) → `tasks.<action>()` →
`NavigationJob` / `BaseMotion` → ports → adapters → Nav2 / TF / cmd_vel.
The execute thread blocks while the goal runs; `MultiThreadedExecutor` keeps
serving cancels, feedback and Nav2 futures on the other threads.

---

## Adding a new action

1. Add `skuba_interfaces/action/<Name>.action` (use `NavigationError error` in
   the result) and rebuild `skuba_interfaces`.
2. Write `tasks.<name>(ctx, ...) -> TaskResult` in `core/navigation/tasks.py`
   using `ctx.navigate(...)` or `ctx.motion...`; add a goal check to
   `validation.py` if needed.
3. Add one `ActionSpec` row to `_action_specs()` in the node.
4. Add tests with the fakes in `tests/navigation_fakes.py`.

It automatically joins the arbiter, cancel handling, feedback throttling and
result/status mapping.

---

## Testing

```bash
cd ~/skuba_ws && colcon build --packages-select skuba_interfaces skuba_navigation
source install/setup.bash
python3 -m pytest src/skuba-navigation/tests/test_navigation_*.py
```

The tests drive the real core code against a simulated robot
(`tests/navigation_fakes.py`: a fake clock plus a base that integrates the
velocity commands), covering retries, cancel / preempt / timeout, TF dropouts,
controller accuracy, standoff geometry, the locations file, and that the
error-code / phase enums match `skuba_interfaces`.
