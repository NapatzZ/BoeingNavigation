"""Pure decision logic for following_server.py. No rclpy import."""

from action_msgs.msg import GoalStatus

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def pose_is_usable(
    person_pose, pose_age_s: float, pose_timeout_s: float, keep_last: bool = False
) -> bool:
    """
    Whether the stored person pose may seed a goal.

    keep_last: any pose received so far will do, however old -- the robot goes to
    where the person was last seen instead of waiting for them to reappear.
    """
    if person_pose is None:
        return False
    return keep_last or pose_age_s <= pose_timeout_s


def goal_status_name(status) -> str:
    return GOAL_STATUS_NAMES.get(status, str(status))
