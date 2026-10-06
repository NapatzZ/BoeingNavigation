"""Unit tests for nav_to_pose.py's pure math/YAML logic.

The retry/TF/cmd_vel control loops themselves stay in the node (they need a
live rclpy graph); this only covers what was extracted as pure functions.
"""

import math

import pytest
from builtin_interfaces.msg import Time

from skuba_navigation.core import nav_to_pose_logic as logic


def test_distance_2d():
    assert logic.distance_2d(0, 0, 3, 4) == pytest.approx(5.0)


def test_compute_rotation_command_sign_matches_error_direction():
    assert logic.compute_rotation_command(math.radians(30.0), 0.0) > 0
    assert logic.compute_rotation_command(-math.radians(30.0), 0.0) < 0


def test_compute_rotation_command_stays_within_speed_limits():
    for angle_diff_deg in (0.5, 5.0, 45.0, 179.0):
        speed = logic.compute_rotation_command(math.radians(angle_diff_deg), 0.0)
        assert (
            logic.ROTATE_MIN_SPEED - 1e-9 <= abs(speed) <= logic.ROTATE_MAX_SPEED + 1e-9
        )


def test_compute_rotation_command_decel_zone_forces_min_speed():
    tiny_error = logic.ROTATE_DECEL_ZONE / 2.0
    speed = logic.compute_rotation_command(tiny_error, error_rate=0.0)
    assert speed == pytest.approx(logic.ROTATE_MIN_SPEED)


def test_compute_rotation_command_large_error_is_capped_at_max_speed():
    speed = logic.compute_rotation_command(math.radians(179.0), error_rate=0.0)
    assert speed == pytest.approx(logic.ROTATE_MAX_SPEED)


def test_path_is_plannable():
    class FakePath:
        def __init__(self, poses):
            self.poses = poses

    assert logic.path_is_plannable(None) is False
    assert logic.path_is_plannable(FakePath([])) is False
    assert logic.path_is_plannable(FakePath([object()])) is True


def test_make_pose_builds_the_expected_fields():
    stamp = Time(sec=1, nanosec=0)
    pose = logic.make_pose(1.0, 2.0, math.pi / 2.0, stamp=stamp, frame_id="map")

    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == pytest.approx(1.0)
    assert pose.pose.position.y == pytest.approx(2.0)
    assert pose.pose.position.z == 0.0
    # A non-zero yaw must produce a non-identity orientation.
    assert pose.pose.orientation.z != pytest.approx(0.0)


def test_verify_directed_standoff_success():
    ok, dist, _ = logic.verify_directed_standoff(
        final_x=1.0,
        final_y=0.0,
        target_x=0.0,
        target_y=0.0,
        requested_side="FRONT",
        standoff=1.0,
    )
    assert ok is True
    assert dist == pytest.approx(1.0)


def test_verify_directed_standoff_wrong_distance():
    ok, dist, msg = logic.verify_directed_standoff(
        final_x=3.0,
        final_y=0.0,
        target_x=0.0,
        target_y=0.0,
        requested_side="FRONT",
        standoff=1.0,
    )
    assert ok is False
    assert dist == pytest.approx(3.0)
    assert "distance" in msg.lower()


def test_verify_directed_standoff_wrong_side():
    # Correct radial distance (1.0m) but on the LEFT side, not the
    # requested FRONT side.
    ok, _dist, msg = logic.verify_directed_standoff(
        final_x=0.0,
        final_y=1.0,
        target_x=0.0,
        target_y=0.0,
        requested_side="FRONT",
        standoff=1.0,
    )
    assert ok is False
    assert "side" in msg.lower()


def test_verify_auto_standoff_within_tolerance():
    ok, dist, _ = logic.verify_auto_standoff(
        final_x=1.0, final_y=0.0, target_x=0.0, target_y=0.0, stop_distance=1.0
    )
    assert ok is True
    assert dist == pytest.approx(1.0)


def test_verify_auto_standoff_outside_tolerance():
    ok, _dist, _ = logic.verify_auto_standoff(
        final_x=2.0, final_y=0.0, target_x=0.0, target_y=0.0, stop_distance=1.0
    )
    assert ok is False


def test_merge_position_starts_empty_when_file_is_missing(tmp_path):
    yaml_path = str(tmp_path / "does_not_exist.yaml")
    data = logic.merge_position(
        yaml_path, "home", "map", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )
    assert data["positions"]["home"]["pose"]["position"]["x"] == pytest.approx(1.0)


def test_write_and_load_saved_positions_roundtrip(tmp_path):
    yaml_path = str(tmp_path / "saved_pose.yaml")
    data = logic.merge_position(
        yaml_path, "home", "map", (1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0)
    )
    logic.write_positions_yaml(yaml_path, data)

    positions = logic.load_saved_positions(yaml_path)
    assert "home" in positions
    assert positions["home"].header.frame_id == "map"
    assert positions["home"].pose.position.x == pytest.approx(1.0)
    assert positions["home"].pose.orientation.w == pytest.approx(1.0)
