# Exercise 5 - Named locations: save places and send goals

**Goal:** stop clicking in RViz. Save named places ("door", "desk", "charger"), then
tell the robot `goto desk` or run a patrol route. You will use the `waypoint_manager`
node in `boeing_navigation` and then read and extend its code.

Works with both modes: saved-map navigation (Exercise 2) and online SLAM + Nav2 (Exercise 4).

> **Waypoints belong to one map.** A waypoint is a pose in the `map` frame. If you re-map the
> room, `map` changes and old waypoints point to the wrong place. Use **one waypoints file per
> map** (e.g. `lab.yaml` -> `lab_waypoints.yaml`). In online-SLAM mode the map can also shift at a
> loop closure, so waypoints saved earlier may drift slightly.

## 5.1 How it works

```
 you ──"save kitchen"──►  /waypoint_manager/command  ──►  waypoint_manager
                                                           │  save: TF  map -> base_link  ──► waypoints.yaml
                                                           │  goto: NavigateToPose action ──► Nav2
                          /waypoint_manager/status   ◄─────┘  markers ──► RViz
```

| Command | What it does |
|---|---|
| `save <name>` | Store the robot's **current** pose (from TF `map -> base_link`) |
| `save_at <name> <x> <y> [deg]` | Store an explicit pose |
| `delete <name>` | Forget a waypoint |
| `list` | Print all waypoints |
| `goto <name>` | Navigate to one waypoint |
| `route <a> <b> <c>` | Visit waypoints in order, then stop |
| `patrol <a> <b> <c>` | Visit waypoints in order, forever |
| `cancel` | Cancel the current goal / route |

Waypoints are saved in a YAML file, so they survive restarts.

## 5.2 Start

Start the robot and Nav2 exactly as in Exercise 2 (saved map) **or** Exercise 4 (SLAM + Nav2),
and make sure the robot is localised (set the initial pose with AMCL). Then:

```bash
# T5 - waypoint manager (one waypoints file per map!)
ros2 launch boeing_navigation waypoints.launch.py \
  waypoints_file:=/ros_ws/src/boeing_navigation/maps/lab_waypoints.yaml
```

In RViz add **MarkerArray** on `/waypoint_manager/markers` (set Durability to
*Transient Local*, Fixed Frame `map`). Saved waypoints show as green arrows with names.

Helper for sending commands (use it from any terminal):

```bash
ros2 run boeing_navigation waypoint_cmd list
```

If nothing prints back, read the result in the `waypoint_manager` terminal or with
`ros2 topic echo /waypoint_manager/status`.

## 5.3 Tasks

**Task 1 - Save places.** Drive the robot with teleop (or send RViz goals) to 4 places and
save each one at the heading you want the robot to finish with:

```bash
ros2 run boeing_navigation waypoint_cmd save door
ros2 run boeing_navigation waypoint_cmd save desk
ros2 run boeing_navigation waypoint_cmd save corner
ros2 run boeing_navigation waypoint_cmd save home
ros2 run boeing_navigation waypoint_cmd list
```

Open the YAML file and read it. Which frame are the numbers in? What unit is `yaw`?

**Task 2 - Go to a place.** Close teleop (only Nav2 may publish `/cmd_vel`), then:

```bash
ros2 run boeing_navigation waypoint_cmd goto desk
```

Watch the distance-remaining log. Does the robot finish with the heading you saved?
Try `cancel` in the middle of a trip.

**Task 3 - Routes and patrol.**

```bash
ros2 run boeing_navigation waypoint_cmd route door desk home
ros2 run boeing_navigation waypoint_cmd patrol door desk corner
ros2 run boeing_navigation waypoint_cmd cancel
```

Send a new `goto` while a route is running. What happens to the old route? (Read
`_stop` and `run_id` in the code to see why.)

**Task 4 - Persistence.** Stop `waypoint_manager` and start it again. Are the waypoints still
there? Delete the waypoints file and restart. Edit the YAML by hand to add a waypoint,
restart, and send the robot there.

**Task 5 - Failure cases.** Predict, then try each one and read the status message:

1. `goto nowhere`
2. `goto desk` while Nav2 is not running
3. `goto` a waypoint inside a wall (use `save_at wall <x> <y>`)
4. `goto desk` while the robot is not localised (no `map -> base_link` TF) -> try `save` too
5. Block the path with a box so the robot cannot get through

**Task 6 - After re-mapping.** Re-map the room (Exercise 1), reuse the old waypoints file, and
check where the waypoints land. Explain what you see, then fix it.

## 5.4 Read the code

Open `boeing_navigation/scripts/waypoint_manager.py` and find:

1. Where the pose is read from TF in `save`, and how the quaternion becomes a yaw angle.
2. Where the `NavigateToPose` goal is built. Which `frame_id` is used and why?
3. How the node knows a goal finished, and how a route moves on to the next waypoint.
4. Why `run_id` exists (what bug would happen without it?).
5. Why the node uses callbacks instead of waiting inside the command handler.

## 5.5 Extend it (pick at least two)

- **Wait at each waypoint:** pause N seconds (a timer, not `time.sleep`) before the next one.
- **`go_home` / `charge`:** add a command that goes to a special `home` waypoint.
- **Loop count:** `patrol a b c 3` repeats the route 3 times.
- **Retry:** if a goal is aborted, retry once (or skip it) instead of stopping the route.
- **Closest-first:** `nearest` goes to the closest saved waypoint.
- **Real ROS interface:** replace the string command topic with a proper service/action
  (custom `.srv` / `.action` in a new interface package).
- **RViz-friendly:** save a waypoint from RViz (e.g. subscribe to `/goal_pose` or `/clicked_point`).
- **Queue goals from a file or a web page** (e.g. a simple button UI or a Foxglove panel).

## Questions

1. Why store waypoints in the `map` frame and not in `odom`?
2. What breaks if the saved waypoint's yaw is wrong? Is it a problem for the path or the final pose?
3. In online-SLAM mode, would you trust waypoints saved an hour ago? Why or why not?
4. Why does `waypoint_cmd` wait for a subscriber before publishing?
5. How would you make the patrol robust against someone standing in the way?

## Deliverables

- Your waypoints YAML for your map and a screenshot of the waypoints in RViz.
- A video or log of `route` / `patrol` running.
- Answers to the tasks and questions, plus the code and a short description of the extension you added.
