"""
Goal lifecycle primitives: cancellation and the single-goal arbiter.

Every base-motion goal (Nav2 or cmd_vel) owns one CancelToken. The
GoalArbiter guarantees only one goal drives the base at a time: a new goal
cancels the running one with PREEMPTED and waits until it has actually stopped
before it starts, so two controllers never command the base together.
"""
import threading
from typing import Optional

from .types import ErrorCode, TaskFailure

_STOP_MESSAGES = {
    ErrorCode.CANCELED: 'Goal canceled.',
    ErrorCode.PREEMPTED: 'Goal preempted by a newer base-motion goal.',
}


class CancelToken:
    """Thread-safe stop request plus a 'finished' signal for hand-over."""

    def __init__(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._finished = threading.Event()
        self._reason: Optional[ErrorCode] = None

    def cancel(self, reason: ErrorCode = ErrorCode.CANCELED) -> None:
        """Request a stop. The first reason wins."""
        with self._lock:
            if self._reason is None:
                self._reason = ErrorCode(reason)
        self._stop.set()

    @property
    def cancelled(self) -> bool:
        return self._stop.is_set()

    @property
    def reason(self) -> Optional[ErrorCode]:
        return self._reason

    def wait(self, seconds: float) -> bool:
        """Sleep up to `seconds`; return True early if cancelled."""
        return self._stop.wait(seconds)

    def raise_if_cancelled(self) -> None:
        if self.cancelled:
            raise TaskFailure(self._reason, _STOP_MESSAGES.get(self._reason, 'Goal stopped.'))

    def mark_finished(self) -> None:
        self._finished.set()

    def wait_finished(self, seconds: float) -> bool:
        return self._finished.wait(seconds)


class GoalArbiter:
    """Allows one active base-motion goal; a newer goal preempts the older."""

    def __init__(self, handover_timeout: float = 5.0):
        self._lock = threading.Lock()
        self._active: Optional[CancelToken] = None
        self._handover_timeout = handover_timeout

    def acquire(self, token: CancelToken) -> None:
        """
        Make `token` the active goal, stopping the previous one first.

        Raises TaskFailure if `token` itself gets preempted while waiting, or
        if the previous goal does not stop within the hand-over timeout.
        """
        with self._lock:
            previous, self._active = self._active, token
        if previous is not None:
            previous.cancel(ErrorCode.PREEMPTED)
            if not previous.wait_finished(self._handover_timeout):
                raise TaskFailure(
                    ErrorCode.UNKNOWN,
                    f'Previous goal did not stop within {self._handover_timeout:.1f}s.')
        token.raise_if_cancelled()

    def release(self, token: CancelToken) -> None:
        token.mark_finished()
        with self._lock:
            if self._active is token:
                self._active = None

    def cancel_active(self, reason: ErrorCode = ErrorCode.CANCELED) -> bool:
        """Stop the active goal, if any. Returns whether there was one."""
        with self._lock:
            active = self._active
        if active is None:
            return False
        active.cancel(reason)
        return True
