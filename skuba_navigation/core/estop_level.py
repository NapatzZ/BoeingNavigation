"""
Live e-stop level from the Kobuki sensor stream. No rclpy import.

The level follows the button: pressed gives True, released gives False. It
is not a latch, so a released button unlocks the base again. If the sensor
stream goes quiet, the level is unknown (None) rather than a stale value.
"""

from typing import Optional

# kobuki_ros_interfaces/SensorState.DIGITAL_INPUT0: the e-stop pin.
DIGITAL_INPUT0 = 1

# Seconds without a sensor message before the level counts as unknown.
STALE_AFTER = 1.0


def is_pressed(digital_input: int) -> bool:
    """Return True if the e-stop pin is set in a SensorState.digital_input."""
    return bool(digital_input & DIGITAL_INPUT0)


class EstopLevel:
    def __init__(self, stale_after: float = STALE_AFTER):
        self._stale_after = stale_after
        self._pressed: Optional[bool] = None
        self._heard_at = 0.0

    def hear(self, digital_input: int, now: float) -> bool:
        """Take one sensor reading. Return True if the level changed."""
        pressed = is_pressed(digital_input)
        changed = pressed != self._pressed
        self._pressed = pressed
        self._heard_at = now
        return changed

    def level(self, now: float) -> Optional[bool]:
        """Return pressed, or None before the first reading or when it is stale."""
        if self._pressed is None or now - self._heard_at > self._stale_after:
            return None
        return self._pressed
