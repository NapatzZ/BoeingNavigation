"""
Unit tests for the pure geometry helpers.

These are pure math with no ROS state, so they run without a robot or a
running graph - which is the whole point of keeping them in geometry_utils.
"""

import math

import pytest

from skuba_navigation.core.geometry_utils import (
    AUTO_SIDES,
    SIDE_OFFSETS,
    approach_pose_for_side,
    normalize_angle,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def test_normalize_angle_wraps_into_pi_range():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    # Range is [-pi, pi], so -pi is the valid representative here.
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(0.5) == pytest.approx(0.5)


def test_yaw_quaternion_roundtrip():
    class Q:
        pass

    for yaw in (0.0, 0.7, -1.9, math.pi / 2.0):
        q = Q()
        q.x = q.y = 0.0
        q.z, q.w = yaw_to_quaternion(yaw)
        assert quaternion_to_yaw(q) == pytest.approx(yaw)


def test_eight_sides_are_defined_45_degrees_apart():
    assert len(SIDE_OFFSETS) == 8
    sorted_offsets = sorted(SIDE_OFFSETS.values())
    for lower, upper in zip(sorted_offsets, sorted_offsets[1:]):
        assert (upper - lower) == pytest.approx(math.pi / 4.0)


@pytest.mark.parametrize(
    "side,expected_xy",
    [
        ("FRONT", (1.0, 0.0)),
        ("LEFT", (0.0, 1.0)),
        ("BACK", (-1.0, 0.0)),
        ("RIGHT", (0.0, -1.0)),
    ],
)
def test_map_relative_sides_land_on_the_expected_axis(side, expected_xy):
    """With reference_yaw=0 the sides are map directions: FRONT = map +X."""
    x, y, _ = approach_pose_for_side(0.0, 0.0, 0.0, side, 1.0)
    assert (x, y) == pytest.approx(expected_xy, abs=1e-9)


@pytest.mark.parametrize("side", sorted(SIDE_OFFSETS))
def test_standoff_pose_faces_back_at_the_target(side):
    """Whatever side we park on, the returned yaw must point at the target."""
    target_x, target_y = 2.5, -1.25
    x, y, yaw = approach_pose_for_side(target_x, target_y, 0.0, side, 1.5)

    bearing_to_target = math.atan2(target_y - y, target_x - x)
    assert normalize_angle(yaw - bearing_to_target) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("side", sorted(SIDE_OFFSETS))
def test_standoff_distance_is_radial_from_the_target(side):
    target_x, target_y = -3.0, 4.0
    standoff = 0.8
    x, y, _ = approach_pose_for_side(target_x, target_y, 0.0, side, standoff)
    assert math.hypot(x - target_x, y - target_y) == pytest.approx(standoff)


def test_object_radius_pushes_the_standoff_further_out():
    """object_radius + stop_distance is how a caller asks to clear an object."""
    stop_distance, object_radius = 1.0, 0.3
    near_x, _, _ = approach_pose_for_side(0.0, 0.0, 0.0, "FRONT", stop_distance)
    far_x, _, _ = approach_pose_for_side(
        0.0, 0.0, 0.0, "FRONT", stop_distance + object_radius
    )
    assert far_x - near_x == pytest.approx(object_radius)


def test_reference_yaw_rotates_the_whole_side_frame():
    """Passing a target yaw makes the sides object-relative instead of map-relative."""
    x, y, _ = approach_pose_for_side(0.0, 0.0, math.pi / 2.0, "FRONT", 1.0)
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_unknown_side_raises():
    with pytest.raises(KeyError):
        approach_pose_for_side(0.0, 0.0, 0.0, "SIDEWAYS", 1.0)


def test_auto_sides_are_not_named_sides():
    """The AUTO sentinels must not collide with a real side name."""
    assert not set(AUTO_SIDES) & set(SIDE_OFFSETS)
