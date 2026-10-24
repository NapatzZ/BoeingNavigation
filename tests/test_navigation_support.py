"""Cancellation/arbiter, standoff geometry, location store, goal validation."""

import math
from pathlib import Path
import threading
import time

import pytest
import yaml

from skuba_navigation.core.navigation import validation
from skuba_navigation.core.navigation.cancellation import CancelToken, GoalArbiter
from skuba_navigation.core.navigation.location_store import (
    Location,
    LocationStore,
    MapLocations,
    NoMapLoaded,
    poses_path_for_map,
    source_map_path,
    write_poses_file,
)
from skuba_navigation.core.navigation.standoff import (
    parse_side,
    standoff_goal,
    verify_standoff,
)
from skuba_navigation.core.navigation.types import ErrorCode, Pose2D, TaskFailure
from skuba_navigation.core.zone_detection import save_location_to_yaml

# ── cancellation ──────────────────────────────────────────────────────────


def test_first_cancel_reason_wins():
    token = CancelToken()
    token.cancel(ErrorCode.PREEMPTED)
    token.cancel(ErrorCode.CANCELED)
    with pytest.raises(TaskFailure) as failure:
        token.raise_if_cancelled()
    assert failure.value.code == ErrorCode.PREEMPTED


def test_arbiter_preempts_and_waits_for_handover():
    arbiter = GoalArbiter(handover_timeout=2.0)
    first, second = CancelToken(), CancelToken()
    arbiter.acquire(first)
    started = threading.Event()
    thread = threading.Thread(target=lambda: (arbiter.acquire(second), started.set()))
    thread.start()
    time.sleep(0.1)
    assert first.reason == ErrorCode.PREEMPTED
    assert not started.is_set()  # waits until the old goal has stopped
    arbiter.release(first)
    thread.join(1.0)
    assert started.is_set()
    assert arbiter.cancel_active() and second.reason == ErrorCode.CANCELED


def test_arbiter_handover_timeout():
    arbiter = GoalArbiter(handover_timeout=0.05)
    arbiter.acquire(CancelToken())
    with pytest.raises(TaskFailure):
        arbiter.acquire(CancelToken())


def test_arbiter_idle_cancel():
    assert GoalArbiter().cancel_active() is False


# ── standoff ──────────────────────────────────────────────────────────────


def test_parse_side():
    assert parse_side("") is None and parse_side(" auto ") is None
    assert parse_side("front_left") == "FRONT_LEFT"
    with pytest.raises(ValueError):
        parse_side("UP")


def test_auto_standoff_goal_is_on_the_robot_line():
    goal = standoff_goal(Pose2D(0, 0, 0), 3.0, 4.0, 1.0, None, 0.0, 0.15, 0.1)
    assert (goal.x, goal.y) == pytest.approx((2.4, 3.2))
    assert goal.yaw == pytest.approx(math.atan2(4, 3))


def test_named_side_goal_faces_target():
    goal = standoff_goal(Pose2D(0, 0, 0), 1.0, 1.0, 0.5, "FRONT", 0.3, 0.15, 0.1)
    assert (goal.x, goal.y, goal.yaw) == pytest.approx((1.8, 1.0, math.pi))


def test_already_parked_returns_none():
    assert standoff_goal(Pose2D(0.5, 0, 0), 1.0, 0.0, 1.0, None, 0.0, 0.15, 0.1) is None
    parked = Pose2D(2.0, 0.0, math.pi)
    assert standoff_goal(parked, 1.0, 0.0, 1.0, "FRONT", 0.0, 0.15, 0.1) is None
    # Inside stop_distance but on the wrong side is NOT parked.
    assert standoff_goal(Pose2D(0.0, 0, 0), 1.0, 0.0, 1.0, "FRONT", 0.0, 0.15, 0.1)


@pytest.mark.parametrize(
    "pose, ok, word",
    [
        (Pose2D(1.0, 0.0), True, "Parked"),
        (Pose2D(3.0, 0.0), False, "distance"),
        (Pose2D(0.0, 1.0), False, "side"),
    ],
)
def test_verify_named_side(pose, ok, word):
    result, _distance, message = verify_standoff(
        pose, 0.0, 0.0, 1.0, "FRONT", 0.0, 0.15, math.radians(22.5)
    )
    assert result is ok and word in message


def test_verify_auto():
    assert verify_standoff(Pose2D(1.0, 0), 0, 0, 1.0, None, 0.0, 0.15, 0.4)[0]
    assert not verify_standoff(Pose2D(2.0, 0), 0, 0, 1.0, None, 0.0, 0.15, 0.4)[0]


# ── location store ────────────────────────────────────────────────────────


def test_location_roundtrip_and_merge_keeps_other_entries(tmp_path):
    path = tmp_path / "saved_pose.yaml"
    path.write_text(
        "positions:\n  door:\n    header: {frame_id: map, side: LEFT}\n"
        "    pose:\n      position: {x: 1, y: 2, z: 0}\n"
        "      orientation: {x: 0, y: 0, z: 0, w: 1}\n"
    )
    store = LocationStore(path)
    assert store.save("home", Location("map", Pose2D(0.5, -0.5, math.pi / 2))) == path
    assert store.names() == ["door", "home"]
    assert store.get("home").pose.yaw == pytest.approx(math.pi / 2)
    assert "side: LEFT" in path.read_text()


def test_location_store_sees_external_edits(tmp_path):
    path = tmp_path / "saved_pose.yaml"
    store = LocationStore(path)
    assert store.get("x") is None
    store.save("a", Location("map", Pose2D(0, 0)))
    other = LocationStore(path)
    other.save("b", Location("map", Pose2D(1, 1)))
    assert store.get("b") is not None
    # Back to back, faster than a coarse mtime tick: still seen at once.
    for name in ("c", "d", "e"):
        save_location_to_yaml(path, name, "FRONT", (0, 0, 0), (0, 0, 0, 1))
        assert store.get(name) is not None


def test_a_failed_write_leaves_the_poses_file_whole(tmp_path):
    path = tmp_path / "a.poses.yaml"
    path.write_text("positions: {}\n")
    path.chmod(0o664)
    with pytest.raises(yaml.YAMLError):
        write_poses_file(path, {"positions": {"x": object()}})
    assert path.read_text() == "positions: {}\n"
    assert list(tmp_path.iterdir()) == [path]  # no stray .tmp
    write_poses_file(path, {"positions": {}})
    assert path.stat().st_mode & 0o777 == 0o664


def test_location_store_rejects_empty_name_and_missing_reload(tmp_path):
    store = LocationStore(tmp_path / "missing.yaml")
    with pytest.raises(ValueError):
        store.save("  ", Location("map", Pose2D(0, 0)))
    with pytest.raises(FileNotFoundError):
        store.reload()


def test_poses_path_sits_next_to_the_map():
    assert poses_path_for_map("/m/30_home/skuba_map_real.yaml") == Path(
        "/m/30_home/skuba_map_real.poses.yaml"
    )


def test_source_map_path_prefers_the_source_checkout(tmp_path):
    source_map = tmp_path / "src" / "maps" / "30_home" / "r.yaml"
    source_map.parent.mkdir(parents=True)
    source_map.write_text("image: r.pgm\n")
    installed = Path(
        "/ws/install/skuba_navigation/share/skuba_navigation/maps/30_home/r.yaml"
    )
    assert source_map_path(installed, tmp_path / "src") == source_map
    assert source_map_path(installed, None) == installed
    other = Path("/ws/install/x/maps/30_home/other.yaml")
    assert source_map_path(other, tmp_path / "src") == other


def test_map_locations_follow_the_running_map(tmp_path):
    running = {"path": tmp_path / "a.poses.yaml"}
    locations = MapLocations(lambda: running["path"])
    locations.save("door", Location("map", Pose2D(1, 2, 0)))
    running["path"] = tmp_path / "b.poses.yaml"
    assert locations.get("door") is None
    saved = locations.save("door", Location("map", Pose2D(9, 9, 0)))
    assert saved == tmp_path / "b.poses.yaml"
    running["path"] = tmp_path / "a.poses.yaml"
    assert locations.get("door").pose.x == 1
    assert locations.path == tmp_path / "a.poses.yaml"
    assert not (tmp_path / "saved_pose.yaml").exists()


def test_map_locations_without_a_map_refuse(tmp_path):
    locations = MapLocations(lambda: None)
    assert locations.path is None
    for call in (
        lambda: locations.get("x"),
        locations.names,
        locations.reload,
        lambda: locations.save("x", Location("map", Pose2D(0, 0))),
    ):
        with pytest.raises(NoMapLoaded):
            call()


# ── validation ────────────────────────────────────────────────────────────


def test_validation():
    assert validation.check_go_to_pose(0, 0, 0, 0, 0, 1) is None
    assert validation.check_go_to_pose(0, 0, 0, 0, 0, 0)
    assert validation.check_go_to_pose(float("nan"), 0, 0, 0, 0, 1)
    assert validation.check_go_to_location("kitchen", True) is None
    assert "not in" in validation.check_go_to_location("kitchen", False)
    assert validation.check_go_to_frame("cup", 0.5, "LEFT", 0.1) is None
    assert "approach_side" in validation.check_go_to_frame("cup", 0.5, "UP", 0.1)
    assert validation.check_go_to_frame("cup", -1.0, "", 0.0)
    assert validation.check_nudge("", 0.5)
    assert validation.check_drive(float("inf"), 0.0)
    assert validation.check_rotate(90.0, 0.0) is None


# ── params file ───────────────────────────────────────────────────────────


def test_params_file_matches_config_defaults():
    import dataclasses
    from pathlib import Path

    from skuba_navigation.core.navigation.config import MotionConfig, NavigationConfig

    path = Path(__file__).resolve().parent.parent / "params" / "navigation_server.yaml"
    params = yaml.safe_load(path.read_text())["navigation_server"]["ros__parameters"]
    for section, cls in (("navigation", NavigationConfig), ("motion", MotionConfig)):
        defaults = {f.name: f.default for f in dataclasses.fields(cls)}
        assert (
            params[section] == defaults
        ), f"params/{path.name} [{section}] out of sync"
