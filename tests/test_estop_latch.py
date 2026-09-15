"""Unit tests for the e-stop latch state machine.

Pure state, no rclpy - so it runs without a robot or a running graph.
"""
from skuba_navigation.core.estop_latch import EstopLatch


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
