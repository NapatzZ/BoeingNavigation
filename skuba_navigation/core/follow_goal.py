"""Pure decision logic for following_server.py. No rclpy import."""

from action_msgs.msg import GoalStatus

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def pose_is_usable(person_pose, pose_age_s: float, pose_timeout_s: float) -> bool:
    """Whether the stored person pose is fresh enough to seed a goal."""
    return person_pose is not None and pose_age_s <= pose_timeout_s


def goal_status_name(status) -> str:
    return GOAL_STATUS_NAMES.get(status, str(status))
