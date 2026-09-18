"""Injectable time: real monotonic time in the node, simulated time in tests."""
import time
from typing import Optional

from .cancellation import CancelToken


class Clock:
    """Monotonic wall clock whose sleep wakes up early on cancel."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float, token: Optional[CancelToken] = None) -> None:
        if token is None:
            time.sleep(max(0.0, seconds))
        else:
            token.wait(max(0.0, seconds))


class Deadline:
    """A point in time after which a goal must give up."""

    def __init__(self, clock: Clock, seconds: float):
        self._clock = clock
        self._end = clock.now() + seconds

    def remaining(self) -> float:
        return max(0.0, self._end - self._clock.now())

    def expired(self) -> bool:
        return self._clock.now() >= self._end


class RateLimiter:
    """Lets an event through at most once per `period` seconds."""

    def __init__(self, clock: Clock, period: float):
        self._clock = clock
        self._period = period
        self._last = None

    def ready(self) -> bool:
        now = self._clock.now()
        if self._last is None or now - self._last >= self._period:
            self._last = now
            return True
        return False

    def mark(self) -> None:
        self._last = self._clock.now()
