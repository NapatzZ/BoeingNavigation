"""Decide what to do after one Nav2 attempt ends."""
from dataclasses import dataclass

from .ports import Nav2Status
from .types import ErrorCode


@dataclass(frozen=True)
class RetryDecision:
    retry: bool
    code: ErrorCode = ErrorCode.NONE
    message: str = ''


def decide(status: Nav2Status, attempt: int, max_retries: int) -> RetryDecision:
    """Only an ABORTED attempt is worth retrying.

    A rejection or an unreachable server will not fix itself in a second, and
    a cancel is a decision someone made - retrying it would override them
    (the old nav_to_pose node did exactly that).
    """
    if status is Nav2Status.SUCCEEDED:
        return RetryDecision(retry=False)
    if status is Nav2Status.ABORTED:
        if attempt <= max_retries:
            return RetryDecision(retry=True)
        return RetryDecision(
            False, ErrorCode.NAVIGATION_FAILED,
            f'Nav2 aborted the goal on all {attempt} attempt(s).')
    if status is Nav2Status.REJECTED:
        return RetryDecision(False, ErrorCode.GOAL_REJECTED, 'Nav2 rejected the goal.')
    if status is Nav2Status.UNAVAILABLE:
        return RetryDecision(
            False, ErrorCode.NAV2_UNAVAILABLE, 'Nav2 navigate_to_pose server is not available.')
    return RetryDecision(False, ErrorCode.CANCELED, 'Nav2 goal was canceled.')
