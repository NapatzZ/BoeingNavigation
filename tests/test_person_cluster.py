"""Unit tests for the person-cluster removal + fixed-grid resample logic.

remove_person_clusters() only reads scalar/list fields off its input (any
duck-typed scan works), but constructs a real sensor_msgs/LaserScan for its
output whenever it finds at least one valid point - so this needs a ROS
environment (sensor_msgs, std_msgs) to run, same as the module itself.
"""
import math

from std_msgs.msg import Header

from skuba_navigation.core.person_cluster import OUT_N, remove_person_clusters


class FakeScan:
    def __init__(self, ranges, angle_min, angle_increment, range_min=0.05, range_max=8.0):
        self.header = Header()
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.time_increment = 0.0
        self.scan_time = 0.0


def test_all_invalid_ranges_returns_the_input_message_unchanged():
    scan = FakeScan(ranges=[math.inf, math.inf, math.nan], angle_min=0.0, angle_increment=0.1)
    out = remove_person_clusters(scan, min_width=0.2, max_width=0.9, cluster_gap=0.2, min_pts=3)
    assert out is scan


def test_output_is_always_resampled_to_the_fixed_grid_length():
    n = 200
    angle_increment = (2 * math.pi) / n
    scan = FakeScan(ranges=[3.0] * n, angle_min=-math.pi, angle_increment=angle_increment)
    out = remove_person_clusters(scan, min_width=0.2, max_width=0.9, cluster_gap=0.2, min_pts=3)
    assert len(out.ranges) == OUT_N


def test_a_person_sized_isolated_cluster_is_blanked():
    from skuba_navigation.core.person_cluster import OUT_ANGLE_INC, OUT_ANGLE_MIN

    # Align the input grid 1:1 with the fixed output grid so nearest-neighbour
    # resampling is an identity mapping - makes the assertion unambiguous.
    ranges = [math.inf] * OUT_N
    center = OUT_N // 2  # angle ~= 0
    person_indices = range(center - 10, center + 10)  # ~20 deg span at r=1.0m
    for i in person_indices:
        ranges[i] = 1.0

    scan = FakeScan(ranges=ranges, angle_min=OUT_ANGLE_MIN, angle_increment=OUT_ANGLE_INC)
    out = remove_person_clusters(scan, min_width=0.25, max_width=0.85, cluster_gap=0.2, min_pts=3)

    assert math.isinf(out.ranges[center])
    # Untouched background stays untouched.
    assert math.isinf(out.ranges[0])


def test_a_cluster_smaller_than_min_pts_is_left_alone_even_if_person_width():
    from skuba_navigation.core.person_cluster import OUT_ANGLE_INC, OUT_ANGLE_MIN

    # Two valid points ~15 deg apart at r=1.0m -> ~0.26m apart, which is
    # "person width" and within cluster_gap, so the only thing that should
    # save them from being blanked is the cluster having < min_pts points.
    ranges = [math.inf] * OUT_N
    center = OUT_N // 2
    ranges[center] = 1.0
    ranges[center + 15] = 1.0  # only 2 points total - below min_pts=3

    scan = FakeScan(ranges=ranges, angle_min=OUT_ANGLE_MIN, angle_increment=OUT_ANGLE_INC)
    out = remove_person_clusters(scan, min_width=0.25, max_width=0.85, cluster_gap=0.3, min_pts=3)

    assert out.ranges[center] == 1.0
    assert out.ranges[center + 15] == 1.0
