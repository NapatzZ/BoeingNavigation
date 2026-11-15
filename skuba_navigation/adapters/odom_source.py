"""OdomSource on the base driver's nav_msgs/Odometry topic."""

import threading
import time

from nav_msgs.msg import Odometry
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from ..core.geometry_utils import quaternion_to_yaw
from ..core.navigation.types import ErrorCode, TaskFailure


class OdomSource:
    def __init__(
        self, node: Node, topic: str, max_age: float, callback_group: CallbackGroup
    ):
        self._topic = topic
        self._max_age = max_age
        self._lock = threading.Lock()
        self._yaw = None
        self._stamp = 0.0
        node.create_subscription(
            Odometry, topic, self._on_odom, 10, callback_group=callback_group
        )

    def _on_odom(self, msg: Odometry) -> None:
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        with self._lock:
            self._yaw, self._stamp = yaw, time.monotonic()

    def yaw(self, timeout: float) -> float:
        end = time.monotonic() + timeout
        while True:
            with self._lock:
                yaw, stamp = self._yaw, self._stamp
            # A stale reading would freeze the remaining angle and spin on.
            if yaw is not None and time.monotonic() - stamp <= self._max_age:
                return yaw
            if time.monotonic() >= end:
                raise TaskFailure(
                    ErrorCode.TF_LOOKUP_FAILED, f"No fresh odometry on {self._topic}."
                )
            time.sleep(0.01)
