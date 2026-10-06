"""Unit tests for MapSaver's path logic and success/failure combination.

No rclpy import in map_saver.py; the actual `ros2 run` / `ros2 service call`
subprocess calls are monkeypatched out so this stays a pure unit test.
"""

import os

from skuba_navigation.core.map_saver import MapSaver


def test_category_is_appended_between_maps_root_and_map_name(tmp_path):
    saver = MapSaver("skuba_home", "30_skuba_home", str(tmp_path))
    assert saver.save_directory == str(tmp_path / "30_skuba_home")
    assert saver.map_path == str(tmp_path / "30_skuba_home" / "skuba_home")


def test_no_category_saves_straight_into_maps_root(tmp_path):
    saver = MapSaver("skuba_home", "", str(tmp_path))
    assert saver.save_directory == str(tmp_path)
    assert saver.map_path == str(tmp_path / "skuba_home")


def test_tilde_in_maps_root_is_expanded_not_taken_literally():
    saver = MapSaver("m", "", "~/some/maps/dir")
    assert "~" not in saver.save_directory
    assert saver.save_directory == os.path.expanduser("~/some/maps/dir")


def test_default_maps_root_is_cwd_when_none_given():
    saver = MapSaver("m")
    assert saver.save_directory == os.getcwd()


def test_save_maps_reports_success_only_when_both_operations_succeed(
    tmp_path, monkeypatch
):
    saver = MapSaver("m", "", str(tmp_path))

    monkeypatch.setattr(saver, "save_nav2_map", lambda: True)
    monkeypatch.setattr(saver, "serialize_slam_map", lambda: True)
    assert saver.save_maps() is True

    monkeypatch.setattr(saver, "save_nav2_map", lambda: True)
    monkeypatch.setattr(saver, "serialize_slam_map", lambda: False)
    assert saver.save_maps() is False

    monkeypatch.setattr(saver, "save_nav2_map", lambda: False)
    monkeypatch.setattr(saver, "serialize_slam_map", lambda: False)
    assert saver.save_maps() is False


def test_save_maps_creates_the_save_directory(tmp_path, monkeypatch):
    saver = MapSaver("m", "new_category", str(tmp_path))
    monkeypatch.setattr(saver, "save_nav2_map", lambda: True)
    monkeypatch.setattr(saver, "serialize_slam_map", lambda: True)

    assert not os.path.isdir(saver.save_directory)
    saver.save_maps()
    assert os.path.isdir(saver.save_directory)


def test_run_command_returns_false_on_subprocess_error(monkeypatch):
    import subprocess

    def boom(*args, **kwargs):
        raise subprocess.CalledProcessError(returncode=1, cmd=args[0])

    monkeypatch.setattr(subprocess, "run", boom)
    saver = MapSaver("m")
    assert (
        saver.run_command(["ros2", "run", "nav2_map_server", "map_saver_cli"]) is False
    )
