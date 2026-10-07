"""
Unit tests for the door open/closed detector.

process_scan() only takes plain ranges/angle numbers, so it needs no ROS
message types at all.
"""

import math

import pytest

from skuba_navigation.core.door_detection import DoorDetector


def _front_cone_detector(open_ratio=0.5, min_beams=3, confirm_need=1, threshold=1.5):
    return DoorDetector(
        threshold=threshold,
        front_center_rad=0.0,
        front_rad=math.radians(30.0),
        open_ratio=open_ratio,
        min_beams=min_beams,
        confirm_need=confirm_need,
    )


def _cone_scan(ranges):
    """4 beams spanning -15..+15 deg, all inside a +-30 deg front cone."""
    return ranges, math.radians(-15.0), math.radians(10.0), 0.2


def test_no_beams_in_forward_cone():
    det = DoorDetector(
        threshold=1.5,
        front_center_rad=0.0,
        front_rad=math.radians(10.0),
        open_ratio=0.5,
        min_beams=1,
        confirm_need=1,
    )
    # All 4 beams land around 180 deg - well outside a 10 deg front cone.
    status, detail = det.process_scan([1.0, 1.0, 1.0, 1.0], math.pi - 0.05, 0.03, 0.2)
    assert status == "no_beams"
    assert detail is None


def test_too_few_valid_beams():
    det = _front_cone_detector(min_beams=10)
    ranges, angle_min, inc, range_min = _cone_scan([2.0, 2.0, 2.0, 2.0])
    status, detail = det.process_scan(ranges, angle_min, inc, range_min)
    assert status == "too_few_beams"
    assert detail == 4


def test_open_ratio_counts_finite_and_infinite_beams_correctly():
    det = _front_cone_detector(
        open_ratio=0.5, min_beams=3, confirm_need=1, threshold=1.5
    )
    ranges, angle_min, inc, range_min = _cone_scan([2.0, 2.0, 0.5, math.inf])
    status, detail = det.process_scan(ranges, angle_min, inc, range_min)

    # valid (finite, >= range_min) = [2.0, 2.0, 0.5] -> open_finite = 2 (>=threshold)
    # open_inf = 1 -> total_open=3, total_count=3+1=4 -> ratio 0.75
    assert status == "state_change"
    state, ratio, total_open, total_count = detail
    assert state == "OPEN"
    assert ratio == pytest.approx(0.75)
    assert total_open == 3
    assert total_count == 4


def test_debounce_requires_confirm_need_consecutive_matching_scans():
    det = _front_cone_detector(
        open_ratio=0.5, min_beams=3, confirm_need=2, threshold=1.5
    )
    ranges, angle_min, inc, range_min = _cone_scan([2.0, 2.0, 2.0, 2.0])

    # First open scan only builds the streak, no state change yet.
    status1, ratio1 = det.process_scan(ranges, angle_min, inc, range_min)
    assert status1 is None
    assert det.door_open is False

    # Second consecutive open scan confirms it.
    status2, detail2 = det.process_scan(ranges, angle_min, inc, range_min)
    assert status2 == "state_change"
    assert detail2[0] == "OPEN"
    assert det.door_open is True


def test_flip_flopping_readings_never_confirm_with_confirm_need_two():
    det = _front_cone_detector(
        open_ratio=0.5, min_beams=3, confirm_need=2, threshold=1.5
    )
    open_scan = _cone_scan([2.0, 2.0, 2.0, 2.0])
    # Valid (>= range_min) but below threshold -> "closed", not "too few beams".
    closed_scan = _cone_scan([0.5, 0.5, 0.5, 0.5])

    for scan in (open_scan, closed_scan, open_scan, closed_scan):
        ranges, angle_min, inc, range_min = scan
        status, _ = det.process_scan(ranges, angle_min, inc, range_min)
        assert status is None
    assert det.door_open is False


def test_set_active_false_resets_confirmation_state():
    det = _front_cone_detector(
        open_ratio=0.5, min_beams=3, confirm_need=2, threshold=1.5
    )
    det.set_active(True)
    ranges, angle_min, inc, range_min = _cone_scan([2.0, 2.0, 2.0, 2.0])
    det.process_scan(ranges, angle_min, inc, range_min)
    det.process_scan(ranges, angle_min, inc, range_min)
    assert det.door_open is True

    det.set_active(False)
    assert det.door_open is False
    assert det.active is False

    # Reactivating starts the debounce streak over - one scan is not enough.
    det.set_active(True)
    status, _ = det.process_scan(ranges, angle_min, inc, range_min)
    assert status is None
    assert det.door_open is False
