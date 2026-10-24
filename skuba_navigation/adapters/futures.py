"""Blocking on an rclpy future from a callback."""

import threading
import time
from typing import Callable, Optional

_POLL = 0.05  # s between should_stop() checks while waiting on a future


def wait_for_future(
    future, should_stop: Callable[[], bool], timeout: Optional[float] = None
) -> bool:
    """Block until `future` is done. False if should_stop() or `timeout` came first."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    start = time.monotonic()
    while not done.wait(_POLL):
        if should_stop():
            return False
        if timeout is not None and time.monotonic() - start > timeout:
            return False
    return True
