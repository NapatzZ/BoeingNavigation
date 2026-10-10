#!/usr/bin/env python3
"""
Nav2 dynamic person-following server (GoalUpdater pattern).

Bridges follow.py (vision) → Nav2 navigation stack the way Nav2 is meant to do
dynamic following (https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html).

Pipeline
  follow.py publishes the person's PoseStamped (map frame) on
    /skuba/nav/person_pose
  This server sends ONE NavigateToPose goal using the follow_point.xml
  behavior tree.  Inside that tree:
    • GoalUpdater reads the SAME person-pose topic and keeps the goal current,
    • ComputePathToPose replans at 2 Hz,
    • TruncatePath keeps the robot DESIRED_DISTANCE behind the person,
    • FollowPath runs continuously (controller never restarts → smooth motion).

Why this replaces the old design
  The previous server re-sent a full NavigateToPose goal at 10 Hz.  Every goal
  restarted the global planner + controller and preempted the previous action,
  so the controller never settled → laggy, jerky, "almost frozen" motion.
  Here we send the goal ONCE and let Nav2 track the moving target internally.

  The follow standoff distance is set by `TruncatePath distance` in
  behavior_trees/follow_point.xml — NOT here.

Author : SKUBA Software Team (2025, rev. 2026)
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from std_srvs.srv import SetBool
from ..core.follow_goal import pose_is_usable, goal_status_name

# ─────────────────────────── tuneable constants ─────────────────────────── #
SEND_RATE_HZ = 2.0  # how often we check whether a goal needs (re)sending
POSE_TIMEOUT_S = 5.0  # s — person considered lost after this with no pose
PACKAGE_NAME = "skuba_navigation"
BT_REL_PATH = "behavior_trees/follow_point.xml"

POSE_TOPIC = "/skuba/nav/person_pose"
SERVICE_NAME = "/skuba/nav/toggle_following"
# ─────────────────────────────────────────────────────────────────────────── #


class FollowingServer(Node):
    """
    Nav2 dynamic person-following server.

    Responsibilities (intentionally small):
      1. Subscribe to the person PoseStamped from follow.py.
      2. On enable, send ONE NavigateToPose goal that runs follow_point.xml.
      3. Re-send that goal only if it terminates (aborted / lost) while enabled.
      4. Cancel on disable.
    Continuous tracking + standoff distance are handled inside the BT by
    GoalUpdater + TruncatePath, so there is no per-pose goal resending here.
    """

    def __init__(self) -> None:
        super().__init__("skuba_following_server")
        self.get_logger().info("Initialising skuba_following_server …")

        # ── State ────────────────────────────────────────────────────────── #
        self._enabled = False
        self._person_pose: PoseStamped | None = None
        self._person_pose_t = 0.0  # perf_counter stamp of last pose
        self._goal_handle = None  # active Nav2 goal handle
        self._goal_active = False  # a goal is sent and not finished
        self._send_in_progress = False  # send_goal_async awaiting acceptance

        # ── Behavior-tree path ───────────────────────────────────────────── #
        self._bt_path = os.path.join(
            get_package_share_directory(PACKAGE_NAME), BT_REL_PATH
        )
        if not os.path.isfile(self._bt_path):
            self.get_logger().error(
                f"Follow behavior tree not found at {self._bt_path} — "
                f"did you add it to setup.py data_files and rebuild?"
            )

        # ── Nav2 NavigateToPose action client ────────────────────────────── #
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── QoS (match follow.py BEST_EFFORT publisher) ──────────────────── #
        _latest_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscription / service / timer ───────────────────────────────── #
        self.create_subscription(PoseStamped, POSE_TOPIC, self._pose_cb, _latest_qos)
        self.create_service(SetBool, SERVICE_NAME, self._svc_cb)
        self.create_timer(1.0 / SEND_RATE_HZ, self._tick)

        self.get_logger().info("skuba_following_server ready.")
        self.get_logger().info(f"  Behavior tree : {self._bt_path}")
        self.get_logger().info(f"  Service       : {SERVICE_NAME}")
        self.get_logger().info(f"  Person pose   : {POSE_TOPIC}")

    # ─────────────────────────── callbacks ──────────────────────────────── #

    def _pose_cb(self, msg: PoseStamped) -> None:
        """Store latest person pose + arrival time (also feeds GoalUpdater)."""
        self._person_pose = msg
        self._person_pose_t = time.perf_counter()

    def _svc_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        """Enable / disable following."""
        self._enabled = req.data
        if not self._enabled:
            self._cancel_goal()
        res.success = True
        res.message = f'FollowingServer {"ENABLED" if req.data else "DISABLED"}'
        self.get_logger().info(res.message)
        return res

    # ─────────────────────────── main tick ──────────────────────────────── #

    def _tick(self) -> None:
        """Send the follow goal once; resend only if it ended while enabled."""
        if not self._enabled or self._goal_active or self._send_in_progress:
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose action server not ready — waiting …",
                throttle_duration_sec=5.0,
            )
            return

        # Need an initial person pose to seed the goal; GoalUpdater takes over after.
        if self._person_pose is None:
            self.get_logger().warn(
                "Enabled but no person pose received yet — waiting …",
                throttle_duration_sec=3.0,
            )
            return
        pose_age = time.perf_counter() - self._person_pose_t
        if not pose_is_usable(self._person_pose, pose_age, POSE_TIMEOUT_S):
            self.get_logger().warn(
                "Person pose is stale — waiting for re-acquisition before sending goal",
                throttle_duration_sec=3.0,
            )
            return

        self._send_goal(self._person_pose)

    # ─────────────────────────── Nav2 plumbing ──────────────────────────── #

    def _send_goal(self, pose: PoseStamped) -> None:
        """Send the single follow goal that runs follow_point.xml."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = self._bt_path  # ← the GoalUpdater follow tree

        self._send_in_progress = True
        self.get_logger().info("Sending follow goal (follow_point.xml) …")
        fut = self._nav_client.send_goal_async(goal_msg)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        self._send_in_progress = False
        if not handle.accepted:
            self.get_logger().warn("Nav2 rejected the follow goal — will retry.")
            return
        self._goal_handle = handle
        self._goal_active = True
        self.get_logger().info("Nav2 accepted follow goal — tracking person.")
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        """Goal finished (succeeded/aborted/canceled). Allow tick to resend."""
        status = future.result().status
        self._goal_active = False
        self._goal_handle = None
        self.get_logger().info(
            f"Follow goal ended: {goal_status_name(status)}"
            + ("  → will resend" if self._enabled else "")
        )

    def _cancel_goal(self) -> None:
        """Cancel the active follow goal if any."""
        self._goal_active = False
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


# ────────────────────────────────── main ────────────────────────────────── #


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowingServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
