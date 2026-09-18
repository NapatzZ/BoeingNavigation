"""Core types <-> ROS messages."""
from typing import Any, Dict

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Pose, PoseStamped
from skuba_interfaces.msg import NavigationProgress

from ..core.geometry_utils import quaternion_to_yaw, yaw_to_quaternion
from ..core.navigation.types import Pose2D, Progress


def pose2d_from_pose(pose: Pose) -> Pose2D:
    return Pose2D(pose.position.x, pose.position.y, quaternion_to_yaw(pose.orientation))


def pose_stamped(pose: Pose2D, frame_id: str, stamp: Time) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.pose.position.x = float(pose.x)
    msg.pose.position.y = float(pose.y)
    msg.pose.orientation.z, msg.pose.orientation.w = yaw_to_quaternion(pose.yaw)
    return msg


def duration_to_seconds(duration: Duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


def progress_msg(progress: Progress, frame_id: str, stamp: Time) -> NavigationProgress:
    msg = NavigationProgress()
    msg.phase = int(progress.phase)
    msg.attempt = int(progress.attempt)
    msg.target_pose = pose_stamped(progress.target, frame_id, stamp)
    if progress.current is not None:
        msg.current_pose = pose_stamped(progress.current, frame_id, stamp)
    msg.distance_remaining = float(progress.distance_remaining)
    msg.estimated_time_remaining = float(progress.estimated_time_remaining)
    msg.number_of_recoveries = int(progress.number_of_recoveries)
    msg.elapsed = float(progress.elapsed)
    return msg


def fill(msg: Any, values: Dict[str, Any], frame_id: str, stamp: Time) -> Any:
    """Copy `values` onto same-named fields of `msg`, converting core types.

    Unknown keys are ignored, so a task can report a field that only some
    actions define (e.g. 'final_distance').
    """
    for name, value in values.items():
        if not hasattr(msg, name) or value is None:
            continue
        if isinstance(value, Pose2D):
            value = pose_stamped(value, frame_id, stamp)
        elif isinstance(value, Progress):
            value = progress_msg(value, frame_id, stamp)
        setattr(msg, name, value)
    return msg
