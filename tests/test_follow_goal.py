"""Unit tests for following_server.py's pure decision helpers."""
from action_msgs.msg import GoalStatus

from skuba_navigation.core.follow_goal import goal_status_name, pose_is_usable


def test_pose_is_usable_requires_a_non_none_pose():
    assert pose_is_usable(None, pose_age_s=0.0, pose_timeout_s=5.0) is False


def test_pose_is_usable_checks_freshness():
    assert pose_is_usable(object(), pose_age_s=1.0, pose_timeout_s=5.0) is True
    assert pose_is_usable(object(), pose_age_s=5.0, pose_timeout_s=5.0) is True  # boundary
    assert pose_is_usable(object(), pose_age_s=5.1, pose_timeout_s=5.0) is False


def test_goal_status_name_known_statuses():
    assert goal_status_name(GoalStatus.STATUS_SUCCEEDED) == 'SUCCEEDED'
    assert goal_status_name(GoalStatus.STATUS_ABORTED) == 'ABORTED'
    assert goal_status_name(GoalStatus.STATUS_CANCELED) == 'CANCELED'


def test_goal_status_name_falls_back_to_str_for_unknown_status():
    assert goal_status_name(999) == '999'
