"""NavigationJob / retry policy: retries, cancel, preempt, timeout, plan check."""
import pytest

from navigation_fakes import Harness
from skuba_navigation.core.navigation.clock import Deadline
from skuba_navigation.core.navigation.config import NavigationConfig
from skuba_navigation.core.navigation.navigation_job import NavigationJob, NavigationRequest
from skuba_navigation.core.navigation.ports import Nav2Status
from skuba_navigation.core.navigation.retry_policy import decide
from skuba_navigation.core.navigation.types import ErrorCode, Phase, Pose2D

GOAL = Pose2D(2.0, 1.0, 0.0)


def run(h, request=NavigationRequest(GOAL), timeout=60.0):
    progress = []
    job = NavigationJob(h.nav2, h.robot, h.nav_config, h.clock, h.token,
                        Deadline(h.clock, timeout), lambda: h.clock.now(),
                        lambda p, force: progress.append((p, force)))
    return job.run(request), progress


def test_decide_retries_only_aborted_within_budget():
    assert decide(Nav2Status.ABORTED, 1, 2).retry
    assert decide(Nav2Status.ABORTED, 2, 2).retry
    final = decide(Nav2Status.ABORTED, 3, 2)
    assert not final.retry and final.code == ErrorCode.NAVIGATION_FAILED


@pytest.mark.parametrize('status, code', [
    (Nav2Status.SUCCEEDED, ErrorCode.NONE),
    (Nav2Status.CANCELED, ErrorCode.CANCELED),
    (Nav2Status.REJECTED, ErrorCode.GOAL_REJECTED),
    (Nav2Status.UNAVAILABLE, ErrorCode.NAV2_UNAVAILABLE),
])
def test_decide_never_retries_other_outcomes(status, code):
    decision = decide(status, 1, 5)
    assert not decision.retry and decision.code == code


def test_success_on_first_attempt():
    h = Harness()
    report, progress = run(h)
    assert report.succeeded and report.attempts == 1
    assert h.robot.pose == GOAL
    assert h.nav2.clears == 1
    phases = [p.phase for p, forced in progress if forced]
    assert phases == [Phase.PLANNING, Phase.NAVIGATING]


def test_aborted_attempts_are_retried_with_costmap_clear():
    h = Harness(script=(Nav2Status.ABORTED, Nav2Status.ABORTED, 'arrive'))
    report, progress = run(h)
    assert report.succeeded and report.attempts == 3
    assert h.nav2.clears == 3   # start + before each retry
    assert Phase.RETRYING in [p.phase for p, _ in progress]


def test_gives_up_after_max_retries():
    h = Harness(script=(Nav2Status.ABORTED,),
                nav_config=NavigationConfig(retry_delay=0.0, max_retries=1))
    report, _ = run(h)
    assert report.code == ErrorCode.NAVIGATION_FAILED
    assert report.attempts == 2


def test_cancel_is_never_retried():
    # The old nav_to_pose node treated a cancel as a failure and retried it.
    h = Harness(script=('hang',))
    h.clock.listeners.append(lambda dt: h.clock.t > 1.0 and h.token.cancel())
    report, _ = run(h)
    assert report.code == ErrorCode.CANCELED
    assert len(h.nav2.goals) == 1


def test_preempt_reports_preempted():
    h = Harness(script=('hang',))
    h.clock.listeners.append(lambda dt: h.clock.t > 1.0 and h.token.cancel(ErrorCode.PREEMPTED))
    report, _ = run(h)
    assert report.code == ErrorCode.PREEMPTED


def test_timeout_is_total_across_attempts():
    h = Harness(script=('hang',))
    report, _ = run(h, timeout=5.0)
    assert report.code == ErrorCode.TIMEOUT
    assert len(h.nav2.goals) == 1
    assert h.clock.now() == pytest.approx(5.0, abs=0.2)


def test_unreachable_goal_fails_before_sending():
    h = Harness(plannable=False)
    report, _ = run(h, NavigationRequest(GOAL, check_plan=True))
    assert report.code == ErrorCode.NO_VALID_PATH
    assert h.nav2.goals == []


def test_unknown_reachability_still_navigates():
    h = Harness(plannable=None)
    report, _ = run(h, NavigationRequest(GOAL, check_plan=True))
    assert report.succeeded


def test_unavailable_and_rejected_map_to_codes():
    assert run(Harness(script=(Nav2Status.UNAVAILABLE,)))[0].code == ErrorCode.NAV2_UNAVAILABLE
    assert run(Harness(script=(Nav2Status.REJECTED,)))[0].code == ErrorCode.GOAL_REJECTED


def test_request_fields_reach_nav2():
    h = Harness()
    run(h, NavigationRequest(GOAL, 'odom', '/bt.xml'))
    assert h.nav2.goals == [(GOAL, 'odom', '/bt.xml')]
