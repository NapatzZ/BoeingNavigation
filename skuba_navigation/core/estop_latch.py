"""
Pure e-stop latch state machine. No rclpy import.

Triggers once, then stays latched permanently — matches the physical
e-stop semantics in emergency_stop.py: never auto-release.
"""


class EstopLatch:
    def __init__(self):
        self._active = False

    @property
    def is_active(self) -> bool:
        return self._active

    def trigger(self) -> bool:
        """Latch the e-stop. Returns True the first time it fires."""
        if self._active:
            return False
        self._active = True
        return True
