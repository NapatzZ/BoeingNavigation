"""
Unit tests for the pure LiDAR angle/distance filter.

filter_scan() only reads .angle_min/.angle_increment/.ranges and deepcopies
whatever it is given, so a plain duck-typed scan (no sensor_msgs import
needed) is enough to exercise it.
"""

import math

import pytest

from skuba_navigation.core.lidar_filter import LidarFilter


class FakeScan:
    def __init__(self, ranges, angle_min=0.0, angle_increment=0.0):
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.ranges = list(ranges)


def test_set_mode_reports_the_current_mode_string():
    lf = LidarFilter()
    assert lf.set_mode(True) == "angle filter  + distance filter"
    assert lf.set_mode(False) == "distance filter only"
    assert lf.enable_angle_filter is False


def test_close_range_is_always_blanked_regardless_of_angle_filter():
    lf = LidarFilter()
    scan = FakeScan(ranges=[0.3, 0.49], angle_min=0.0, angle_increment=0.0)
    out = lf.filter_scan(scan)
    assert math.isinf(out.ranges[0])
    assert math.isinf(out.ranges[1])


def test_angle_filter_off_leaves_far_ranges_untouched():
    lf = LidarFilter()
    lf.set_mode(False)
    scan = FakeScan(ranges=[2.0], angle_min=0.0, angle_increment=0.0)
    out = lf.filter_scan(scan)
    assert out.ranges[0] == pytest.approx(2.0)


def test_angle_filter_on_blanks_the_front_sector():
    lf = LidarFilter()
    lf.set_mode(True)
    # angle 0 deg falls inside the 0-100 deg front-sector branch.
    scan = FakeScan(ranges=[2.0], angle_min=0.0, angle_increment=0.0)
    out = lf.filter_scan(scan)
    assert math.isinf(out.ranges[0])


def test_angle_filter_on_leaves_the_side_sector_untouched():
    lf = LidarFilter()
    lf.set_mode(True)
    # 180 deg is outside both front sectors (260-360 and 0-100).
    scan = FakeScan(ranges=[2.0], angle_min=math.pi, angle_increment=0.0)
    out = lf.filter_scan(scan)
    assert out.ranges[0] == pytest.approx(2.0)


def test_non_finite_input_ranges_are_left_alone():
    lf = LidarFilter()
    scan = FakeScan(ranges=[math.inf, math.nan], angle_min=0.0, angle_increment=0.0)
    out = lf.filter_scan(scan)
    assert math.isinf(out.ranges[0])
    assert math.isnan(out.ranges[1])


def test_filter_scan_does_not_mutate_the_input():
    lf = LidarFilter()
    scan = FakeScan(ranges=[0.1], angle_min=0.0, angle_increment=0.0)
    lf.filter_scan(scan)
    assert scan.ranges[0] == pytest.approx(0.1)
