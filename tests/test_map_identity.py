"""Unit tests for core/navigation/map_identity.py (which saved map is loaded)."""

from pathlib import Path

from map_fakes import make_map

from skuba_navigation.core.navigation.map_identity import (
    MapShape,
    find_running_map,
    shape_of_map_yaml,
)

HOME = dict(width=4, height=3, origin=(-1.0, -2.0, 0.0))


def shape(width=4, height=3, origin=(-1.0, -2.0)):
    return MapShape(width, height, 0.05, origin[0], origin[1])


def test_shape_of_a_saved_map(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    assert shape_of_map_yaml(tmp_path / "30" / "a.yaml") == shape()
    assert shape_of_map_yaml(tmp_path / "30" / "missing.yaml") is None


def test_the_reported_map_wins_when_it_matches(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    make_map(tmp_path, "30/b", **HOME)
    reported = tmp_path / "30" / "b.yaml"
    assert find_running_map(shape(), reported, tmp_path) == reported


def test_a_swapped_map_is_found_when_one_map_matches(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    make_map(tmp_path, "30/b", width=6, height=3, origin=(-3.0, -2.0, 0.0))
    stale = tmp_path / "30" / "a.yaml"
    live = shape(width=6, origin=(-3.0, -2.0))
    assert find_running_map(live, stale, tmp_path) == tmp_path / "30" / "b.yaml"


def test_an_unclear_swap_is_refused(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    stale = tmp_path / "30" / "a.yaml"
    live = shape(width=9)
    assert find_running_map(live, stale, tmp_path) is None  # no map matches
    make_map(tmp_path, "30/b", width=9, height=3, origin=(-1.0, -2.0, 0.0))
    make_map(tmp_path, "30/c", width=9, height=3, origin=(-1.0, -2.0, 0.0))
    assert find_running_map(live, stale, tmp_path) is None  # two maps match


def test_an_unreadable_reported_map_cannot_be_checked(tmp_path):
    gone = tmp_path / "30" / "gone.yaml"
    assert find_running_map(shape(), gone, tmp_path) == gone


def test_an_unreadable_reported_map_gives_way_to_the_one_that_matches(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    (tmp_path / "30" / "a.poses.yaml").write_text("positions: {}\n")  # not a map
    gone = tmp_path / "30" / "gone.yaml"
    assert find_running_map(shape(), gone, tmp_path) == tmp_path / "30" / "a.yaml"


def test_an_unreadable_reported_map_with_several_matches_is_refused(tmp_path):
    make_map(tmp_path, "30/a", **HOME)
    make_map(tmp_path, "30/b", **HOME)
    gone = tmp_path / "30" / "gone.yaml"
    assert find_running_map(shape(), gone, tmp_path) is None


def test_every_committed_map_can_be_identified():
    """A map whose shape cannot be read is never recognised after a swap."""
    maps_root = Path(__file__).resolve().parent.parent / "maps"
    maps = [
        path
        for path in sorted(maps_root.rglob("*.yaml"))
        if not path.name.endswith(".poses.yaml")
    ]
    assert maps
    unreadable = [str(path) for path in maps if shape_of_map_yaml(path) is None]
    assert not unreadable
