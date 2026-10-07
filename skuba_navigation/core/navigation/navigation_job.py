"""
Send one target to Nav2 and see it through.

This is the one pipeline behind every Nav2-backed action:

    clear costmaps -> (optional) reachability check -> send goal
      -> relay feedback -> on ABORTED: clear + retry until max_retries
      -> stop at once on cancel / preempt / deadline
"""
from dataclasses import dataclass
from typing import Callable

from .cancellation import CancelToken
from .clock import Clock, Deadline
from .config import NavigationConfig
from .ports import Nav2Feedback, Nav2Port, PoseSource
from .retry_policy import decide
from .types import ErrorCode, Phase, Pose2D, Progress, TaskFailure

ProgressSink = Callable[[Progress, bool], None]   # (progress, force_publish)


@dataclass(frozen=True)
class NavigationRequest:
    target: Pose2D
    frame_id: str = 'map'
    behavior_tree: str = ''
    check_plan: bool = False   # fail fast with NO_VALID_PATH if the planner cannot reach it


@dataclass(frozen=True)
class NavigationReport:
    code: ErrorCode
    message: str
    attempts: int

    @property
    def succeeded(self) -> bool:
        return self.code == ErrorCode.NONE


class NavigationJob:
    """Runs one NavigationRequest against Nav2 for one goal."""

    def __init__(self, nav2: Nav2Port, poses: PoseSource, config: NavigationConfig,
                 clock: Clock, token: CancelToken, deadline: Deadline,
                 elapsed: Callable[[], float], on_progress: ProgressSink):
        self._nav2 = nav2
        self._poses = poses
        self._config = config
        self._clock = clock
        self._token = token
        self._deadline = deadline
        self._elapsed = elapsed
        self._on_progress = on_progress

    def run(self, request: NavigationRequest) -> NavigationReport:
        attempt = 0
        try:
            self._report(Phase.PLANNING, request, attempt)
            if self._config.clear_costmaps_on_start:
                self._nav2.clear_costmaps()
            if request.check_plan:
                self._require_path(request)

            while True:
                self._raise_if_stopped()
                attempt += 1
                self._report(Phase.NAVIGATING, request, attempt)
                status = self._nav2.navigate(
                    request.target, request.frame_id, request.behavior_tree,
                    on_feedback=lambda fb, n=attempt: self._relay(request, n, fb),
                    should_stop=self._should_stop)
                # A stop we asked for comes back as Nav2 CANCELED; report the
                # real reason (CANCELED / PREEMPTED / TIMEOUT) instead.
                self._raise_if_stopped()

                decision = decide(status, attempt, self._config.max_retries)
                if not decision.retry:
                    return NavigationReport(decision.code, decision.message, attempt)

                self._report(Phase.RETRYING, request, attempt)
                self._clock.sleep(self._config.retry_delay, self._token)
                if self._config.clear_costmaps_on_retry:
                    self._nav2.clear_costmaps()
        except TaskFailure as failure:
            return NavigationReport(failure.code, failure.message, attempt)

    def _require_path(self, request: NavigationRequest) -> None:
        start = self._poses.robot_pose()
        if self._nav2.is_plannable(start, request.target, request.frame_id) is False:
            target = request.target
            raise TaskFailure(
                ErrorCode.NO_VALID_PATH,
                f'No path to ({target.x:.2f}, {target.y:.2f}); '
                'it is unreachable or in collision.')

    def _should_stop(self) -> bool:
        return self._token.cancelled or self._deadline.expired()

    def _raise_if_stopped(self) -> None:
        self._token.raise_if_cancelled()
        if self._deadline.expired():
            raise TaskFailure(ErrorCode.TIMEOUT,
                              f'Goal timed out after {self._elapsed():.1f}s.')

    def _report(self, phase: Phase, request: NavigationRequest, attempt: int) -> None:
        self._on_progress(
            Progress(phase=phase, attempt=attempt, target=request.target,
                     elapsed=self._elapsed()),
            True)

    def _relay(self, request: NavigationRequest, attempt: int, fb: Nav2Feedback) -> None:
        self._on_progress(
            Progress(phase=Phase.NAVIGATING, attempt=attempt, target=request.target,
                     elapsed=self._elapsed(), current=fb.pose,
                     distance_remaining=fb.distance_remaining,
                     estimated_time_remaining=fb.estimated_time_remaining,
                     number_of_recoveries=fb.number_of_recoveries),
            False)
