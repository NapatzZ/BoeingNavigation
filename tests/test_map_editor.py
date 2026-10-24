"""Unit tests for core/map_editor.py (PGM I/O, paths, coordinates, locations)."""

import math
from pathlib import Path

import pytest
import yaml

from map_fakes import make_map
from skuba_navigation.core import map_editor as ed
from skuba_navigation.core.navigation.location_store import LocationStore


def test_pgm_round_trip(tmp_path):
    px = bytearray([0, 205, 254, 7, 8, 9])
    ed.write_pgm(tmp_path / "m.pgm", 3, 2, px)
    assert ed.read_pgm(tmp_path / "m.pgm") == (3, 2, px)


def test_reads_pgm_with_comment_header(tmp_path):
    (tmp_path / "m.pgm").write_bytes(b"P5\n# CREATOR: map_saver\n2 1\n255\n\x00\xfe")
    assert ed.read_pgm(tmp_path / "m.pgm") == (2, 1, bytearray(b"\x00\xfe"))


def test_reads_ascii_pgm(tmp_path):
    (tmp_path / "m.pgm").write_bytes(b"P2\n# 8-bit pgm\n3 1\n255\n0 205\n254\n")
    assert ed.read_pgm(tmp_path / "m.pgm") == (3, 1, bytearray([0, 205, 254]))


def test_truncated_ascii_pgm_is_rejected(tmp_path):
    (tmp_path / "m.pgm").write_bytes(b"P2\n3 1\n255\n0 205\n")
    with pytest.raises(ValueError):
        ed.read_pgm(tmp_path / "m.pgm")


def test_list_and_load_map(tmp_path):
    px = make_map(tmp_path)
    (tmp_path / "30_home" / "notes.yaml").write_text("a: 1\n")  # not a map
    assert ed.list_maps(tmp_path) == ["30_home/home"]
    m = ed.load_map(tmp_path, "30_home/home")
    assert (m["width"], m["height"], m["resolution"]) == (4, 3, 0.05)
    assert m["pixels"] == px


@pytest.mark.parametrize("bad", ["../etc/passwd", "/etc/passwd", "30_home/../../x", ""])
def test_map_id_cannot_escape_maps_root(tmp_path, bad):
    with pytest.raises(ValueError):
        ed.resolve_map(tmp_path, bad)


def test_save_keeps_first_original_as_bak(tmp_path):
    original = make_map(tmp_path)
    ed.save_map_pixels(tmp_path, "30_home/home", 4, 3, bytes(12))
    ed.save_map_pixels(tmp_path, "30_home/home", 4, 3, bytes([254]) * 12)
    folder = tmp_path / "30_home"
    assert ed.read_pgm(folder / "home.pgm.bak")[2] == original
    assert ed.read_pgm(folder / "home.pgm")[2] == bytearray([254]) * 12


def test_save_rejects_size_change(tmp_path):
    make_map(tmp_path)
    with pytest.raises(ValueError):
        ed.save_map_pixels(tmp_path, "30_home/home", 5, 3, bytes(15))


def test_pixel_world_round_trip():
    origin, res, height = [-1.0, -2.0, 0.0], 0.05, 3
    # bottom-left cell centre sits half a cell inside the origin
    assert ed.pixel_to_world(0, 2, height, res, origin) == pytest.approx(
        (-0.975, -1.975)
    )
    x, y = ed.pixel_to_world(3, 0, height, res, origin)
    assert ed.world_to_pixel(x, y, height, res, origin) == pytest.approx((3, 0))


def test_poses_path_is_the_maps_sidecar_and_stays_inside_maps(tmp_path):
    assert ed.poses_path(tmp_path, "30_home/skuba_map_real") == (
        tmp_path / "30_home" / "skuba_map_real.poses.yaml"
    )
    with pytest.raises(ValueError):
        ed.poses_path(tmp_path, "../x")


def test_each_map_keeps_its_own_locations(tmp_path):
    make_map(tmp_path, "30_home/a")
    make_map(tmp_path, "30_home/b")
    ed.save_locations(
        ed.poses_path(tmp_path, "30_home/a"),
        [{"name": "start", "x": 1.0, "y": 1.0, "yaw": 0.0}],
    )
    assert ed.load_locations(ed.poses_path(tmp_path, "30_home/b")) == []
    assert not (tmp_path / "saved_pose.yaml").exists()


def test_locations_round_trip_matches_location_store(tmp_path):
    path = tmp_path / "saved_pose.yaml"
    locs = [
        {"name": "kitchen", "x": 1.5, "y": -0.25, "yaw": math.pi / 2},
        {"name": "door", "x": 0.0, "y": 3.0, "yaw": -2.0},
    ]
    ed.save_locations(path, locs)
    back = {loc["name"]: loc for loc in ed.load_locations(path)}  # dump sorts names
    assert set(back) == {"kitchen", "door"}
    for want in locs:
        got = back[want["name"]]
        assert (got["x"], got["y"], got["yaw"]) == pytest.approx(
            (want["x"], want["y"], want["yaw"])
        )
    # the reader navigation_server actually uses
    kitchen = LocationStore(path).get("kitchen")
    assert kitchen.frame_id == "map"
    assert (kitchen.pose.x, kitchen.pose.y, kitchen.pose.yaw) == pytest.approx(
        (1.5, -0.25, math.pi / 2)
    )


def test_save_keeps_fields_the_editor_does_not_own(tmp_path):
    path = tmp_path / "saved_pose.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "positions": {
                    "kitchen": {
                        "header": {"frame_id": "map", "side": "north"},
                        "pose": {
                            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        },
                    },
                    "old": {"header": {"frame_id": "map"}, "pose": {}},
                }
            }
        )
    )
    ed.save_locations(path, [{"name": "kitchen", "x": 2.0, "y": 1.0, "yaw": 0.0}])
    data = yaml.safe_load(path.read_text())
    assert set(data["positions"]) == {"kitchen"}  # deleted in the editor -> gone
    assert data["positions"]["kitchen"]["header"]["side"] == "north"
    assert data["positions"]["kitchen"]["pose"]["position"]["x"] == 2.0


def test_save_locations_backs_up_previous_file(tmp_path):
    path = tmp_path / "saved_pose.yaml"
    path.write_text("positions: {}\n")
    ed.save_locations(path, [{"name": "a", "x": 0, "y": 0, "yaw": 0}])
    assert len(list(tmp_path.glob("saved_pose.yaml.*.bak"))) == 1


@pytest.mark.parametrize(
    "locs",
    [
        [{"name": " ", "x": 0, "y": 0, "yaw": 0}],
        [
            {"name": "a", "x": 0, "y": 0, "yaw": 0},
            {"name": "a", "x": 1, "y": 1, "yaw": 0},
        ],
    ],
)
def test_save_locations_rejects_blank_or_duplicate_names(tmp_path, locs):
    with pytest.raises(ValueError):
        ed.save_locations(tmp_path / "saved_pose.yaml", locs)


def test_missing_locations_file_is_empty(tmp_path):
    assert ed.load_locations(tmp_path / "nope.yaml") == []


def test_committed_poses_files_belong_to_a_map_and_load():
    maps_root = Path(__file__).resolve().parent.parent / "maps"
    poses = sorted(maps_root.rglob("*.poses.yaml"))
    assert poses
    for path in poses:
        map_yaml = path.with_name(path.name.replace(".poses.yaml", ".yaml"))
        assert map_yaml.exists(), f"{path.name} has no map next to it"
        assert LocationStore(path).names()
