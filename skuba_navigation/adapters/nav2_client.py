"""
Nav2Port on Nav2's own action/service interfaces.

Replaces nav2_simple_commander.BasicNavigator, which is a separate Node that
spins itself and keeps one global "current task" - unsafe to drive from the
callbacks of another node and shared by every request.
"""

import threading
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from ..core.navigation.ports import Nav2Feedback, Nav2Status
from ..core.navigation.types import Pose2D
from .conversions import duration_to_seconds, pose2d_from_pose, pose_stamped

_POLL = 0.05  # s between should_stop() checks while waiting on a future

_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: Nav2Status.SUCCEEDED,
    GoalStatus.STATUS_CANCELED: Nav2Status.CANCELED,
    GoalStatus.STATUS_ABORTED: Nav2Status.ABORTED,
}


class Nav2Client:
    def __init__(
        self,
        node: Node,
        callback_group: CallbackGroup,
        *,
        navigate_action: str,
        compute_path_action: str,
        clear_costmap_services: list,
        server_timeout: float,
        cancel_timeout: float,
    ):
        self._node = node
        self._log = node.get_logger()
        self._server_timeout = server_timeout
        self._cancel_timeout = cancel_timeout
        self._navigate = ActionClient(
            node, NavigateToPose, navigate_action, callback_group=callback_group
        )
        self._compute_path = ActionClient(
            node, ComputePathToPose, compute_path_action, callback_group=callback_group
        )
        self._clear_clients = [
            node.create_client(ClearEntireCostmap, name, callback_group=callback_group)
            for name in clear_costmap_services
        ]

    def navigate(
        self,
        goal: Pose2D,
        frame_id: str,
        behavior_tree: str,
        on_feedback: Callable[[Nav2Feedback], None],
        should_stop: Callable[[], bool],
    ) -> Nav2Status:
        if not self._navigate.wait_for_server(timeout_sec=self._server_timeout):
            return Nav2Status.UNAVAILABLE

        request = NavigateToPose.Goal()
        request.pose = pose_stamped(
            goal, frame_id, self._node.get_clock().now().to_msg()
        )
        request.behavior_tree = behavior_tree
        send_future = self._navigate.send_goal_async(
            request, feedback_callback=lambda msg: on_feedback(_feedback(msg.feedback))
        )

        if not _wait(send_future, should_stop, self._server_timeout):
            # Stopped (or no answer) before Nav2 accepted: cancel it if it lands later.
            send_future.add_done_callback(_cancel_if_accepted)
            return Nav2Status.CANCELED if should_stop() else Nav2Status.UNAVAILABLE
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return Nav2Status.REJECTED

        result_future = handle.get_result_async()
        if _wait(result_future, should_stop):
            return _STATUS.get(result_future.result().status, Nav2Status.ABORTED)

        handle.cancel_goal_async()
        if not _wait(result_future, lambda: False, self._cancel_timeout):
            self._log.warning(
                f"Nav2 did not confirm the cancel within {self._cancel_timeout:.1f}s."
            )
        return Nav2Status.CANCELED

    def is_plannable(
        self, start: Pose2D, goal: Pose2D, frame_id: str
    ) -> Optional[bool]:
        if not self._compute_path.wait_for_server(timeout_sec=self._server_timeout):
            self._log.warning(
                "compute_path_to_pose unavailable; skipping reachability check."
            )
            return None
        stamp = self._node.get_clock().now().to_msg()
        request = ComputePathToPose.Goal()
        request.start = pose_stamped(start, frame_id, stamp)
        request.goal = pose_stamped(goal, frame_id, stamp)
        request.use_start = True
        send_future = self._compute_path.send_goal_async(request)
        if not _wait(send_future, lambda: False, self._server_timeout):
            return None
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        if not _wait(result_future, lambda: False, 2.0 * self._server_timeout):
            handle.cancel_goal_async()
            return None
        response = result_future.result()
        return (
            response.status == GoalStatus.STATUS_SUCCEEDED
            and len(response.result.path.poses) > 0
        )

    def clear_costmaps(self) -> bool:
        ok = True
        for client in self._clear_clients:
            if not client.wait_for_service(timeout_sec=self._server_timeout):
                self._log.warning(
                    f"{client.srv_name} unavailable; costmap not cleared."
                )
                ok = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            if not _wait(future, lambda: False, self._server_timeout):
                self._log.warning(f"{client.srv_name} did not answer.")
                ok = False
        return ok


def _wait(
    future, should_stop: Callable[[], bool], timeout: Optional[float] = None
) -> bool:
    """Block until `future` is done. False if should_stop() or `timeout` came first."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    start = time.monotonic()
    while not done.wait(_POLL):
        if should_stop():
            return False
        if timeout is not None and time.monotonic() - start > timeout:
            return False
    return True


def _cancel_if_accepted(send_future) -> None:
    handle = send_future.result()
    if handle is not None and handle.accepted:
        handle.cancel_goal_async()


def _feedback(msg) -> Nav2Feedback:
    return Nav2Feedback(
        pose=pose2d_from_pose(msg.current_pose.pose),
        distance_remaining=float(msg.distance_remaining),
        estimated_time_remaining=duration_to_seconds(msg.estimated_time_remaining),
        number_of_recoveries=int(msg.number_of_recoveries),
    )
