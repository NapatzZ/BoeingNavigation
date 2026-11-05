"""
Unit tests for the e-stop latch state machine.

Pure state, no rclpy - so it runs without a robot or a running graph.
"""

from skuba_navigation.core.estop_latch import DIGITAL_INPUT0, EstopLatch, is_pressed


def test_starts_inactive():
    latch = EstopLatch()
    assert latch.is_active is False


def test_trigger_activates_and_returns_true_the_first_time():
    latch = EstopLatch()
    assert latch.trigger() is True
    assert latch.is_active is True


def test_trigger_is_idempotent_once_latched():
    """Matches the physical e-stop: never auto-release, re-triggering is a no-op."""
    latch = EstopLatch()
    latch.trigger()
    assert latch.trigger() is False
    assert latch.trigger() is False
    assert latch.is_active is True


def test_is_pressed_reads_only_pin_0_of_the_sensor_word():
    assert is_pressed(DIGITAL_INPUT0) is True
    assert is_pressed(0) is False
    assert is_pressed(0b1110) is False
    assert is_pressed(0b1111) is True
