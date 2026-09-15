"""Unit tests for save_location.py's zone-accumulation + saved-position YAML logic."""
import math

import pytest
import yaml

from skuba_navigation.core import zone_detection


class FakeScan:
    def __init__(self, ranges, angle_min, angle_increment, range_min):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min


def test_new_zone_accumulator_starts_all_zones_at_zero():
    acc = zone_detection.new_zone_accumulator()
    assert set(acc) == set(zone_detection.ZONES)
    assert all(v == 0 for v in acc.values())


def test_best_zone_is_none_when_everything_is_zero():
    acc = zone_detection.new_zone_accumulator()
    assert zone_detection.best_zone(acc) == "NONE"


def test_best_zone_picks_the_highest_count():
    acc = zone_detection.new_zone_accumulator()
    acc["LEFT"] = 5
    acc["BACK"] = 2
    assert zone_detection.best_zone(acc) == "LEFT"


def test_accumulate_zone_hits_attributes_a_single_beam_to_the_front_zone():
    # TF base_link->laser is yaw=pi, so a raw laser beam at -pi maps to the
    # FRONT zone (0 deg) in the robot frame - see the module docstring math.
    scan = FakeScan(ranges=[0.5], angle_min=-math.pi, angle_increment=0.0, range_min=0.1)
    acc = zone_detection.new_zone_accumulator()
    zone_detection.accumulate_zone_hits(scan, acc)

    assert acc["FRONT"] == 1
    for side in zone_detection.ZONES:
        if side != "FRONT":
            assert acc[side] == 0


def test_accumulate_zone_hits_ignores_ranges_outside_the_obstacle_window():
    # range_min above the beam's range -> not counted as a hit anywhere.
    scan = FakeScan(ranges=[0.5], angle_min=-math.pi, angle_increment=0.0, range_min=0.6)
    acc = zone_detection.new_zone_accumulator()
    zone_detection.accumulate_zone_hits(scan, acc)
    assert sum(acc.values()) == 0

    # range beyond OBSTACLE_RANGE -> not counted either.
    far_scan = FakeScan(
        ranges=[zone_detection.OBSTACLE_RANGE + 1.0], angle_min=-math.pi,
        angle_increment=0.0, range_min=0.1)
    acc2 = zone_detection.new_zone_accumulator()
    zone_detection.accumulate_zone_hits(far_scan, acc2)
    assert sum(acc2.values()) == 0


def test_yaw_for_side_adds_the_side_offset():
    from skuba_navigation.core.geometry_utils import SIDE_OFFSETS
    current = 0.3
    assert zone_detection.yaw_for_side(current, "LEFT") == pytest.approx(
        current + SIDE_OFFSETS["LEFT"])


def test_yaw_for_side_unknown_side_is_a_zero_offset():
    assert zone_detection.yaw_for_side(0.3, "NONE") == pytest.approx(0.3)


def test_save_location_to_yaml_writes_and_returns_the_merged_data(tmp_path):
    yaml_path = tmp_path / "saved_pose.yaml"
    data = zone_detection.save_location_to_yaml(
        yaml_path, "kitchen", "LEFT", (1.0, 2.0, 0.0), (0.0, 0.0, 0.1, 0.99))

    assert data["positions"]["kitchen"]["header"]["side"] == "LEFT"
    assert yaml_path.exists()

    on_disk = yaml.safe_load(yaml_path.read_text())
    assert on_disk["positions"]["kitchen"]["pose"]["position"]["x"] == pytest.approx(1.0)


def test_save_location_to_yaml_merges_with_existing_entries(tmp_path):
    yaml_path = tmp_path / "saved_pose.yaml"
    zone_detection.save_location_to_yaml(yaml_path, "a", "FRONT", (0, 0, 0), (0, 0, 0, 1))
    data = zone_detection.save_location_to_yaml(yaml_path, "b", "BACK", (1, 1, 0), (0, 0, 0, 1))

    assert set(data["positions"]) == {"a", "b"}
