#!/usr/bin/env python3
"""Save named locations and send Nav2 goals to them.

Commands are sent as text on /waypoint_manager/command (std_msgs/String), usually
through the `waypoint_cmd` helper:

    save <name>                    save the robot's current pose (global_frame) as <name>
    save_at <name> <x> <y> [deg]   save an explicit pose
    delete <name>                  forget a waypoint
    list                           print all waypoints
    goto <name>                    navigate to one waypoint
    route <a> <b> <c> ...          visit waypoints in order, then stop
    patrol <a> <b> <c> ...         visit waypoints in order, forever
    cancel                         cancel the current goal / route

Waypoints are stored in a YAML file (parameter `waypoints_file`) so they survive restarts.
Status text is published on /waypoint_manager/status and the saved waypoints are shown
as arrows + labels on /waypoint_manager/markers (add a MarkerArray display in RViz).
"""
import math
import os
import shlex

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter(
            'waypoints_file', os.path.expanduser('~/.ros/boeing_waypoints.yaml'))
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.file = os.path.expanduser(
            self.get_parameter('waypoints_file').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.waypoints = {}          # name -> {'x': float, 'y': float, 'yaw': float (rad)}
        self._load()

        # Route execution state. run_id invalidates callbacks of older goals.
        self.run_id = 0
        self.route = []
        self.index = 0
        self.patrol = False
        self.goal_handle = None
        self._last_feedback = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.status_pub = self.create_publisher(String, '~/status', 10)
        marker_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', marker_qos)
        self.create_subscription(String, '~/command', self._on_command, 10)

        self._publish_markers()
        self.say(f'Ready. {len(self.waypoints)} waypoint(s) loaded from {self.file}. '
                 f'Frames: {self.global_frame} <- {self.base_frame}')

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _load(self):
        if not os.path.exists(self.file):
            return
        try:
            with open(self.file, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.waypoints = {
                str(k): {'x': float(v['x']), 'y': float(v['y']), 'yaw': float(v.get('yaw', 0.0))}
                for k, v in (data.get('waypoints') or {}).items()}
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError) as e:
            self.get_logger().error(f'Could not read {self.file}: {e}')

    def _save(self):
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.file)), exist_ok=True)
            tmp = self.file + '.tmp'
            with open(tmp, 'w') as f:
                yaml.safe_dump({'frame_id': self.global_frame, 'waypoints': self.waypoints},
                               f, default_flow_style=False, sort_keys=True)
            os.replace(tmp, self.file)
        except OSError as e:
            self.say(f'ERROR: could not write {self.file}: {e}')
            return False
        return True

    def _publish_markers(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, (name, wp) in enumerate(sorted(self.waypoints.items())):
            arrow = Marker()
            arrow.header.frame_id = self.global_frame
            arrow.header.stamp = now
            arrow.ns = 'waypoint_arrow'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
            arrow.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.4, 0.08, 0.08
            arrow.color.g, arrow.color.a = 0.9, 1.0
            array.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self.global_frame
            label.header.stamp = now
            label.ns = 'waypoint_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = wp['x']
            label.pose.position.y = wp['y']
            label.pose.position.z = 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name
            array.markers.append(label)
        self.marker_pub.publish(array)

    # ----------------------------------------------------------------- commands
    def _on_command(self, msg):
        try:
            parts = shlex.split(msg.data)
        except ValueError as e:
            self.say(f'ERROR: cannot parse command "{msg.data}": {e}')
            return
        if not parts:
            return
        cmd, args = parts[0].lower(), parts[1:]

        handlers = {
            'save': self._cmd_save,
            'save_at': self._cmd_save_at,
            'delete': self._cmd_delete,
            'list': self._cmd_list,
            'goto': lambda a: self._cmd_route(a[:1] if a else a, patrol=False),
            'route': lambda a: self._cmd_route(a, patrol=False),
            'patrol': lambda a: self._cmd_route(a, patrol=True),
            'cancel': self._cmd_cancel,
        }
        handler = handlers.get(cmd)
        if handler is None:
            self.say(f'ERROR: unknown command "{cmd}". Use: {", ".join(handlers)}')
            return
        handler(args)

    def _cmd_save(self, args):
        if len(args) != 1:
            self.say('Usage: save <name>')
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, Time())
        except TransformException as e:
            self.say(f'ERROR: no transform {self.global_frame} -> {self.base_frame} '
                     f'(is SLAM/AMCL running and localised?): {e}')
            return
        tr = t.transform
        self._store(args[0], tr.translation.x, tr.translation.y,
                    yaw_from_quaternion(tr.rotation))

    def _cmd_save_at(self, args):
        try:
            if len(args) not in (3, 4):
                raise ValueError
            yaw = math.radians(float(args[3])) if len(args) == 4 else 0.0
            self._store(args[0], float(args[1]), float(args[2]), yaw)
        except ValueError:
            self.say('Usage: save_at <name> <x> <y> [yaw_degrees]')

    def _store(self, name, x, y, yaw):
        existed = name in self.waypoints
        self.waypoints[name] = {'x': x, 'y': y, 'yaw': yaw}
        if self._save():
            self._publish_markers()
            self.say(f'{"Updated" if existed else "Saved"} "{name}": '
                     f'x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')

    def _cmd_delete(self, args):
        if len(args) != 1:
            self.say('Usage: delete <name>')
            return
        if args[0] not in self.waypoints:
            self.say(f'ERROR: no waypoint "{args[0]}"')
            return
        del self.waypoints[args[0]]
        if self._save():
            self._publish_markers()
            self.say(f'Deleted "{args[0]}"')

    def _cmd_list(self, _args):
        if not self.waypoints:
            self.say('No waypoints saved.')
            return
        lines = [f'  {n}: x={w["x"]:.2f} y={w["y"]:.2f} yaw={math.degrees(w["yaw"]):.0f} deg'
                 for n, w in sorted(self.waypoints.items())]
        self.say('Waypoints:\n' + '\n'.join(lines))

    def _cmd_cancel(self, _args):
        self._stop()
        self.say('Cancelled.')

    def _cmd_route(self, names, patrol):
        if not names:
            self.say('Usage: goto <name> | route <a> <b> ... | patrol <a> <b> ...')
            return
        missing = [n for n in names if n not in self.waypoints]
        if missing:
            self.say(f'ERROR: unknown waypoint(s): {", ".join(missing)}. Use "list".')
            return
        if not self.client.server_is_ready():
            self.say('ERROR: navigate_to_pose action server not available - is Nav2 running?')
            return
        self._stop()
        self.route = list(names)
        self.index = 0
        self.patrol = patrol
        self.say(f'{"Patrolling" if patrol else "Route"}: {" -> ".join(names)}')
        self._send_next()

    # --------------------------------------------------------- goal execution
    def _stop(self):
        """Invalidate the current run and cancel any active goal."""
        self.run_id += 1
        self.route = []
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _send_next(self):
        if self.index >= len(self.route):
            if self.patrol and self.route:
                self.index = 0
            else:
                self.say('Route complete.')
                self.route = []
                self.goal_handle = None
                return
        name = self.route[self.index]
        wp = self.waypoints[name]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        run = self.run_id
        self.say(f'Going to "{name}" ({self.index + 1}/{len(self.route)})')
        future = self.client.send_goal_async(
            goal, feedback_callback=lambda fb: self._on_feedback(run, name, fb))
        future.add_done_callback(lambda f: self._on_goal_response(run, name, f))

    def _on_goal_response(self, run, name, future):
        handle = future.result()
        if run != self.run_id:           # a newer command replaced this one
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.say(f'Goal "{name}" was rejected by Nav2. Route stopped.')
            self.route = []
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(run, name, f))

    def _on_feedback(self, run, name, feedback_msg):
        if run != self.run_id:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_feedback >= 2.0:
            self._last_feedback = now
            dist = feedback_msg.feedback.distance_remaining
            self.get_logger().info(f'"{name}": {dist:.2f} m remaining')

    def _on_result(self, run, name, future):
        if run != self.run_id:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.say(f'Reached "{name}".')
            self.index += 1
            self._send_next()
            return
        reason = {GoalStatus.STATUS_ABORTED: 'aborted by Nav2 (no path / stuck?)',
                  GoalStatus.STATUS_CANCELED: 'cancelled'}.get(status, f'status {status}')
        self.say(f'Goal "{name}" {reason}. Route stopped.')
        self.route = []
        self.goal_handle = None


def main():
    rclpy.init()
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
