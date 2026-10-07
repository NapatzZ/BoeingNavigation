#!/usr/bin/env python3
"""
navigation_server - SKUBA navigation actions on top of Nav2.

ROS glue only. Each action is one row in `_action_specs()`: how to validate
its goal and which core task runs it. The behaviour lives in
core/navigation/tasks.py; ROS I/O lives in adapters/.

Actions (all share one GoalArbiter - a new goal preempts the running one):
  /skuba/nav/go_to_pose       GoToPose        Nav2
  /skuba/nav/go_to_location   GoToLocation    Nav2, saved location
  /skuba/nav/go_to_frame      GoToFrame       Nav2, standoff next to a TF
  /skuba/nav/rotate           RotateInPlace   cmd_vel
  /skuba/nav/drive_straight   DriveStraight   cmd_vel
  /skuba/nav/nudge_to_frame   NudgeToFrame    cmd_vel
Services:
  /skuba/nav/clear_costmaps    std_srvs/Trigger
  /skuba/nav/save_location     SaveLocationCommand  current pose -> locations file
  /skuba/nav/reload_locations  std_srvs/Trigger
"""

from dataclasses import dataclass
from pathlib import Path
import signal
import threading
from typing import Any, Callable, Dict, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from skuba_interfaces.action import (
    DriveStraight,
    GoToFrame,
    GoToLocation,
    GoToPose,
    NudgeToFrame,
    RotateInPlace,
)
from skuba_interfaces.srv import SaveLocationCommand
from std_srvs.srv import Trigger

from ..adapters.cmd_vel_base import CmdVelBase
from ..adapters.conversions import fill, pose2d_from_pose
from ..adapters.nav2_client import Nav2Client
from ..adapters.parameters import declare_dataclass
from ..adapters.tf_pose_source import TfPoseSource
from ..core.navigation import tasks, validation
from ..core.navigation.cancellation import CancelToken, GoalArbiter
from ..core.navigation.clock import Clock, RateLimiter
from ..core.navigation.config import MotionConfig, NavigationConfig
from ..core.navigation.location_store import (
    Location,
    LocationStore,
    resolve_locations_file,
)
from ..core.navigation.motion import BaseMotion
from ..core.navigation.types import ErrorCode, TaskFailure, TaskResult
from ..core.package_paths import find_package_source_dir

NAMESPACE = "/skuba/nav"
PACKAGE_NAME = "skuba_navigation"


@dataclass(frozen=True)
class ActionSpec:
    name: str
    action_type: Any
    validate: Callable[[Any], Optional[str]]  # goal -> rejection reason
    run: Callable[[tasks.TaskContext, Any], TaskResult]  # (ctx, goal) -> result


class NavigationServer(Node):
    def __init__(self):
        super().__init__("navigation_server")
        self._nav_config = declare_dataclass(self, "navigation", NavigationConfig)
        self._motion_config = declare_dataclass(self, "motion", MotionConfig)
        self._map_frame = self._param("map_frame", "map")

        clients_group = ReentrantCallbackGroup()
        self._goals_group = ReentrantCallbackGroup()
        self._core_clock = Clock()
        self._nav2 = Nav2Client(
            self,
            clients_group,
            navigate_action=self._param("nav2.navigate_action", "navigate_to_pose"),
            compute_path_action=self._param(
                "nav2.compute_path_action", "compute_path_to_pose"
            ),
            clear_costmap_services=self._param(
                "nav2.clear_costmap_services",
                [
                    "global_costmap/clear_entirely_global_costmap",
                    "local_costmap/clear_entirely_local_costmap",
                ],
            ),
            server_timeout=self._param("nav2.server_timeout", 2.0),
            cancel_timeout=self._param("nav2.cancel_timeout", 5.0),
        )
        self._poses = TfPoseSource(
            self, self._map_frame, self._param("robot_frame", "base_link")
        )
        self._base = CmdVelBase(
            self,
            self._param("cmd_vel_topic", "cmd_vel_nav_manual"),
            self._motion_config.stop_repeats,
        )
        self._motion = BaseMotion(
            self._poses, self._base, self._motion_config, self._core_clock
        )
        self._locations = LocationStore(
            resolve_locations_file(
                self._param("locations_file", ""), _default_locations_files()
            )
        )
        self._arbiter = GoalArbiter(self._param("preempt_handover_timeout", 5.0))
        self._tokens: Dict[bytes, CancelToken] = {}
        self._tokens_lock = threading.Lock()

        self._action_servers = [self._serve(spec) for spec in self._action_specs()]
        self._services = [
            self.create_service(
                Trigger,
                f"{NAMESPACE}/clear_costmaps",
                self._on_clear_costmaps,
                callback_group=clients_group,
            ),
            self.create_service(
                SaveLocationCommand,
                f"{NAMESPACE}/save_location",
                self._on_save_location,
                callback_group=clients_group,
            ),
            self.create_service(
                Trigger,
                f"{NAMESPACE}/reload_locations",
                self._on_reload_locations,
                callback_group=clients_group,
            ),
        ]
        self.get_logger().info(
            f"navigation_server ready; locations file: {self._locations.path} "
            f"({len(self._locations.names())} locations)."
        )

    # ── action table ──────────────────────────────────────────────────────

    def _action_specs(self):
        def pose_values(goal):
            p = goal.pose.pose
            return (
                p.position.x,
                p.position.y,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )

        return [
            ActionSpec(
                "go_to_pose",
                GoToPose,
                validate=lambda g: validation.check_go_to_pose(*pose_values(g)),
                run=lambda ctx, g: tasks.go_to_pose(
                    ctx,
                    pose2d_from_pose(g.pose.pose),
                    g.pose.header.frame_id,
                    g.timeout,
                    g.behavior_tree,
                ),
            ),
            ActionSpec(
                "go_to_location",
                GoToLocation,
                validate=lambda g: validation.check_go_to_location(
                    g.location, self._locations.get(g.location) is not None
                ),
                run=lambda ctx, g: tasks.go_to_location(
                    ctx, g.location, g.timeout, g.behavior_tree
                ),
            ),
            ActionSpec(
                "go_to_frame",
                GoToFrame,
                validate=lambda g: validation.check_go_to_frame(
                    g.frame, g.stop_distance, g.approach_side, g.object_radius
                ),
                run=lambda ctx, g: tasks.go_to_frame(
                    ctx,
                    g.frame,
                    g.stop_distance,
                    g.approach_side,
                    g.object_radius,
                    g.timeout,
                    g.behavior_tree,
                ),
            ),
            ActionSpec(
                "rotate",
                RotateInPlace,
                validate=lambda g: validation.check_rotate(
                    g.angle_deg, g.tolerance_deg
                ),
                run=lambda ctx, g: tasks.rotate_in_place(
                    ctx, g.angle_deg, g.tolerance_deg, g.timeout
                ),
            ),
            ActionSpec(
                "drive_straight",
                DriveStraight,
                validate=lambda g: validation.check_drive(g.distance, g.speed),
                run=lambda ctx, g: tasks.drive_straight(
                    ctx, g.distance, g.speed, g.timeout
                ),
            ),
            ActionSpec(
                "nudge_to_frame",
                NudgeToFrame,
                validate=lambda g: validation.check_nudge(g.frame, g.stop_distance),
                run=lambda ctx, g: tasks.nudge_to_frame(
                    ctx, g.frame, g.stop_distance, g.timeout
                ),
            ),
        ]

    # ── action plumbing (identical for every action) ──────────────────────

    def _serve(self, spec: ActionSpec) -> ActionServer:
        return ActionServer(
            self,
            spec.action_type,
            f"{NAMESPACE}/{spec.name}",
            execute_callback=lambda handle: self._execute(spec, handle),
            goal_callback=lambda goal: self._on_goal(spec, goal),
            handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel,
            callback_group=self._goals_group,
        )

    def _on_goal(self, spec: ActionSpec, goal) -> GoalResponse:
        try:
            reason = spec.validate(goal)
        except Exception as e:  # a validator must never take the server down
            reason = f"validation error: {e}"
        if reason is None:
            return GoalResponse.ACCEPT
        self.get_logger().warning(f"{spec.name}: goal rejected - {reason}")
        return GoalResponse.REJECT

    def _on_accepted(self, handle) -> None:
        # Register the token before execution so an early cancel is not lost.
        with self._tokens_lock:
            self._tokens[bytes(handle.goal_id.uuid)] = CancelToken()
        handle.execute()

    def _on_cancel(self, handle) -> CancelResponse:
        with self._tokens_lock:
            token = self._tokens.get(bytes(handle.goal_id.uuid))
        if token is not None:
            token.cancel(ErrorCode.CANCELED)
        return CancelResponse.ACCEPT

    def _execute(self, spec: ActionSpec, handle):
        key = bytes(handle.goal_id.uuid)
        with self._tokens_lock:
            token = self._tokens[key]
        started_at = self._core_clock.now()
        ctx = tasks.TaskContext(
            nav2=self._nav2,
            poses=self._poses,
            motion=self._motion,
            locations=self._locations,
            nav_config=self._nav_config,
            motion_config=self._motion_config,
            clock=self._core_clock,
            token=token,
            feedback=self._feedback_sink(spec, handle, started_at),
            started_at=started_at,
        )
        try:
            self._arbiter.acquire(token)
            self.get_logger().info(f"{spec.name}: started.")
            outcome = spec.run(ctx, handle.request)
        except TaskFailure as failure:
            outcome = TaskResult(failure.code, failure.message)
        except Exception as e:
            self.get_logger().error(f"{spec.name}: unexpected error: {e!r}")
            outcome = TaskResult(ErrorCode.UNKNOWN, f"{type(e).__name__}: {e}")
        finally:
            self._arbiter.release(token)
            with self._tokens_lock:
                self._tokens.pop(key, None)
        return self._finish(spec, handle, outcome, started_at)

    def _finish(self, spec: ActionSpec, handle, outcome: TaskResult, started_at: float):
        result = fill(
            spec.action_type.Result(),
            {**outcome.fields, "elapsed": self._core_clock.now() - started_at},
            self._map_frame,
            self.get_clock().now().to_msg(),
        )
        result.error.code = int(outcome.code)
        result.error.message = outcome.message

        # rclpy requires one severity per logging call site, hence the
        # separate calls rather than picking a log function dynamically.
        summary = f"{spec.name}: {outcome.code.name} - {outcome.message}"
        if outcome.succeeded:
            handle.succeed()
            self.get_logger().info(summary)
        elif outcome.code == ErrorCode.CANCELED and handle.is_cancel_requested:
            handle.canceled()
            self.get_logger().info(summary)
        elif outcome.code in (ErrorCode.CANCELED, ErrorCode.PREEMPTED):
            handle.abort()
            self.get_logger().info(summary)
        else:
            handle.abort()
            self.get_logger().warning(summary)
        return result

    def _feedback_sink(self, spec: ActionSpec, handle, started_at: float):
        limiter = RateLimiter(
            self._core_clock, 1.0 / max(self._nav_config.feedback_rate, 0.1)
        )

        def publish(values: dict, force: bool) -> None:
            if force:
                limiter.mark()
            elif not limiter.ready():
                return
            values = {"elapsed": self._core_clock.now() - started_at, **values}
            handle.publish_feedback(
                fill(
                    spec.action_type.Feedback(),
                    values,
                    self._map_frame,
                    self.get_clock().now().to_msg(),
                )
            )

        return publish

    # ── services ──────────────────────────────────────────────────────────

    def _on_clear_costmaps(self, _request, response):
        response.success = self._nav2.clear_costmaps()
        response.message = (
            "Costmaps cleared." if response.success else "Some clears failed."
        )
        return response

    def _on_save_location(self, request, response):
        try:
            pose = self._poses.robot_pose()
            self._locations.save(request.location_name, Location(self._map_frame, pose))
            self.get_logger().info(
                f"Saved '{request.location_name.strip()}' at "
                f"({pose.x:.2f}, {pose.y:.2f}) in {self._locations.path}."
            )
            response.success = True
        except (TaskFailure, ValueError, OSError) as e:
            self.get_logger().error(
                f"save_location '{request.location_name}' failed: {e}"
            )
            response.success = False
        return response

    def _on_reload_locations(self, _request, response):
        try:
            count = self._locations.reload()
            response.success = True
            response.message = f"Loaded {count} locations from {self._locations.path}."
        except Exception as e:
            response.success = False
            response.message = f"Could not load {self._locations.path}: {e}"
        return response

    # ── lifecycle ─────────────────────────────────────────────────────────

    def stop_motion(self) -> None:
        """Shutdown path: stop any goal and command zero velocity."""
        self._arbiter.cancel_active(ErrorCode.CANCELED)
        try:
            self._base.stop()
        except Exception as e:
            self.get_logger().error(f"Could not send the stop command on shutdown: {e}")

    def _param(self, name: str, default):
        return self.declare_parameter(name, default).value


def _default_locations_files():
    """
    Where saved_pose.yaml is looked for when `locations_file` is not set.

    The source checkout is preferred so saved locations survive `colcon build`
    (same lookup as save_location: any folder name or nesting); the install
    share is the fallback.
    """
    share = Path(get_package_share_directory(PACKAGE_NAME))
    source_dir = find_package_source_dir(share, PACKAGE_NAME)
    candidates = [share / "saved_pose.yaml"]
    if source_dir is not None:
        candidates.insert(0, source_dir / "share" / "saved_pose.yaml")
    return candidates


def _raise_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves: rclpy's handler would invalidate the
    # context before `finally` runs, and the zero-velocity stop could not be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _raise_keyboard_interrupt)
    node = NavigationServer()
    # Goals block their thread while waiting on Nav2 / running a control loop;
    # the other threads keep serving cancels, feedback and Nav2 futures.
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        # spin_once with a timeout (not spin()) so Ctrl-C is delivered promptly.
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_motion()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
