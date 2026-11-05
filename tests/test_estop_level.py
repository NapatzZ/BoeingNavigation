"""
Unit tests for the live e-stop level.

Pure state, no rclpy - so it runs without a robot or a running graph.
"""

from skuba_navigation.core.estop_level import DIGITAL_INPUT0, EstopLevel, is_pressed


def test_is_pressed_reads_only_pin_0():
    assert is_pressed(DIGITAL_INPUT0) is True
    assert is_pressed(0) is False
    assert is_pressed(0b1110) is False
    assert is_pressed(0b1111) is True


def test_unknown_before_the_first_reading():
    assert EstopLevel().level(now=0.0) is None


def test_first_reading_sets_the_level_and_counts_as_a_change():
    est = EstopLevel()
    assert est.hear(0, now=0.0) is True
    assert est.level(now=0.0) is False


def test_button_held_at_start_reads_pressed():
    est = EstopLevel()
    est.hear(DIGITAL_INPUT0, now=0.0)
    assert est.level(now=0.0) is True


def test_same_reading_is_not_a_change():
    est = EstopLevel()
    est.hear(0, now=0.0)
    assert est.hear(0, now=0.02) is False


def test_release_goes_back_to_false():
    """Not a latch: a released button unlocks the base again."""
    est = EstopLevel()
    est.hear(DIGITAL_INPUT0, now=0.0)
    assert est.hear(0, now=0.5) is True
    assert est.level(now=0.5) is False


def test_press_after_release_is_a_change():
    est = EstopLevel()
    est.hear(0, now=0.0)
    assert est.hear(DIGITAL_INPUT0, now=0.5) is True
    assert est.level(now=0.5) is True


def test_unknown_when_the_stream_goes_quiet():
    est = EstopLevel(stale_after=1.0)
    est.hear(DIGITAL_INPUT0, now=0.0)
    assert est.level(now=1.0) is True
    assert est.level(now=1.01) is None


def test_a_new_reading_after_silence_brings_the_level_back():
    est = EstopLevel(stale_after=1.0)
    est.hear(0, now=0.0)
    assert est.level(now=5.0) is None
    est.hear(0, now=5.0)
    assert est.level(now=5.0) is False
