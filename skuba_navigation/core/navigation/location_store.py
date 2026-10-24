"""
Named locations, one poses file per map.

A map's poses sit next to it: maps/<folder>/<map>.poses.yaml for
maps/<folder>/<map>.yaml. A pose is only meaningful in the frame of the map it
was saved on, so MapLocations always reads and writes the running map's file.

File format (shared with the save_location node and the map editor):

    positions:
      kitchen:
        header: {frame_id: map}          # save_location also writes `side`
        pose:
          position: {x: .., y: .., z: ..}
          orientation: {x: .., y: .., z: .., w: ..}

Other writers edit the same file, so reads re-load it whenever its mtime
changes, and writes merge into what is on disk (keeping unknown keys) and
replace the file atomically.
"""

from dataclasses import dataclass
import os
from pathlib import Path
import tempfile
import threading
from typing import Callable, Dict, List, Optional

import yaml

from ..geometry_utils import quaternion_to_yaw, yaw_to_quaternion
from .types import Pose2D


@dataclass(frozen=True)
class Location:
    frame_id: str
    pose: Pose2D


class NoMapLoaded(Exception):
    """Poses belong to a map and none is running."""


def poses_path_for_map(map_yaml) -> Path:
    return Path(map_yaml).with_suffix(".poses.yaml")


def source_map_path(map_yaml, source_dir: Optional[Path]) -> Path:
    """
    The running map's yaml, in the source checkout when it has that map.

    map_server may load an installed copy; poses saved next to it would be lost
    on the next build, so write beside the checkout's copy instead.
    """
    map_yaml = Path(map_yaml)
    if source_dir is None or "maps" not in map_yaml.parts:
        return map_yaml
    parts = map_yaml.parts
    inside_maps = len(parts) - parts[::-1].index("maps")
    candidate = Path(source_dir, "maps", *parts[inside_maps:])
    return candidate if candidate.exists() else map_yaml


class LocationStore:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.Lock()
        self._mtime: Optional[float] = None
        self._locations: Dict[str, Location] = {}

    def get(self, name: str) -> Optional[Location]:
        with self._lock:
            self._refresh()
            return self._locations.get(name.strip())

    def names(self) -> List[str]:
        with self._lock:
            self._refresh()
            return sorted(self._locations)

    def reload(self) -> int:
        """Force a re-read. Returns the number of locations. Raises on a bad file."""
        with self._lock:
            self._mtime = None
            self._refresh(strict=True)
            return len(self._locations)

    def save(self, name: str, location: Location) -> None:
        name = name.strip()
        if not name:
            raise ValueError("Location name must not be empty.")
        with self._lock:
            data = self._read_raw()
            qz, qw = yaw_to_quaternion(location.pose.yaw)
            data.setdefault("positions", {})[name] = {
                "header": {"frame_id": location.frame_id},
                "pose": {
                    "position": {"x": location.pose.x, "y": location.pose.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
            }
            self._write_raw(data)
            self._mtime = None
            self._refresh()

    def _refresh(self, strict: bool = False) -> None:
        try:
            mtime = self.path.stat().st_mtime
        except FileNotFoundError:
            if strict:
                raise
            self._locations, self._mtime = {}, None
            return
        if mtime == self._mtime:
            return
        self._locations = _parse(self._read_raw())
        self._mtime = mtime

    def _read_raw(self) -> dict:
        if not self.path.exists():
            return {"positions": {}}
        with open(self.path, "r") as f:
            data = yaml.safe_load(f) or {}
        data.setdefault("positions", {})
        return data

    def _write_raw(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            os.replace(tmp, self.path)
        except BaseException:
            os.unlink(tmp)
            raise


class _Quaternion:
    def __init__(self, d: dict):
        self.x, self.y = d.get("x", 0.0), d.get("y", 0.0)
        self.z, self.w = d.get("z", 0.0), d.get("w", 1.0)


def _parse(data: dict) -> Dict[str, Location]:
    locations = {}
    for name, entry in (data.get("positions") or {}).items():
        position = entry["pose"]["position"]
        yaw = quaternion_to_yaw(_Quaternion(entry["pose"]["orientation"]))
        frame_id = (entry.get("header") or {}).get("frame_id") or "map"
        locations[str(name)] = Location(
            frame_id, Pose2D(position["x"], position["y"], yaw)
        )
    return locations


class MapLocations:
    """
    The locations of whichever map is running, looked up on every call.

    `current_map` returns the running map's yaml path, or None when no map is
    loaded; then every call raises NoMapLoaded instead of guessing a map.
    """

    def __init__(self, current_map: Callable[[], Optional[Path]]):
        self._current_map = current_map
        self._stores: Dict[Path, LocationStore] = {}

    @property
    def path(self) -> Optional[Path]:
        map_yaml = self._current_map()
        return None if map_yaml is None else poses_path_for_map(map_yaml)

    def get(self, name: str) -> Optional[Location]:
        return self._store().get(name)

    def names(self) -> List[str]:
        return self._store().names()

    def reload(self) -> int:
        return self._store().reload()

    def save(self, name: str, location: Location) -> None:
        self._store().save(name, location)

    def _store(self) -> LocationStore:
        path = self.path
        if path is None:
            raise NoMapLoaded("no map loaded; poses belong to a saved map.")
        if path not in self._stores:
            self._stores[path] = LocationStore(path)
        return self._stores[path]
