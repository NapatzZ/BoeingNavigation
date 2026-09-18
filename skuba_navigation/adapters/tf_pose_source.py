"""PoseSource on tf2."""
from typing import Optional, Tuple

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ..core.geometry_utils import quaternion_to_yaw
from ..core.navigation.types import ErrorCode, Pose2D, TaskFailure


class TfPoseSource:

    def __init__(self, node: Node, map_frame: str, robot_frame: str,
                 default_timeout: float = 1.0):
        self._map_frame = map_frame
        self._robot_frame = robot_frame
        self._default_timeout = default_timeout
        self._buffer = Buffer()
        # Own spin thread: TF keeps flowing while our control loops block
        # executor threads.
        self._listener = TransformListener(self._buffer, node, spin_thread=True)

    def robot_pose(self, timeout: Optional[float] = None) -> Pose2D:
        t = self._lookup(self._robot_frame, timeout)
        return Pose2D(t.translation.x, t.translation.y, quaternion_to_yaw(t.rotation))

    def frame_position(self, frame: str, timeout: Optional[float] = None) -> Tuple[float, float]:
        t = self._lookup(frame, timeout)
        return t.translation.x, t.translation.y

    def _lookup(self, frame: str, timeout: Optional[float]):
        seconds = self._default_timeout if timeout is None else timeout
        try:
            return self._buffer.lookup_transform(
                self._map_frame, frame, Time(), Duration(seconds=seconds)).transform
        except TransformException as e:
            raise TaskFailure(ErrorCode.TF_LOOKUP_FAILED,
                              f'No transform {self._map_frame} -> {frame}: {e}') from e
