"""
Named locations persisted in saved_pose.yaml.

File format (shared with the save_location node and pose_picker_web):

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
from typing import Dict, List, Optional, Sequence

import yaml

from ..geometry_utils import quaternion_to_yaw, yaw_to_quaternion
from .types import Pose2D


@dataclass(frozen=True)
class Location:
    frame_id: str
    pose: Pose2D


def resolve_locations_file(configured: str, candidates: Sequence[Path]) -> Path:
    """Pick the configured path, else the first candidate whose folder exists."""
    if configured.strip():
        return Path(configured).expanduser()
    for candidate in candidates:
        if candidate.parent.is_dir():
            return candidate
    return candidates[-1]


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
            raise ValueError('Location name must not be empty.')
        with self._lock:
            data = self._read_raw()
            qz, qw = yaw_to_quaternion(location.pose.yaw)
            data.setdefault('positions', {})[name] = {
                'header': {'frame_id': location.frame_id},
                'pose': {
                    'position': {'x': location.pose.x, 'y': location.pose.y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw},
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
            return {'positions': {}}
        with open(self.path, 'r') as f:
            data = yaml.safe_load(f) or {}
        data.setdefault('positions', {})
        return data

    def _write_raw(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)
            os.replace(tmp, self.path)
        except BaseException:
            os.unlink(tmp)
            raise


class _Quaternion:
    def __init__(self, d: dict):
        self.x, self.y = d.get('x', 0.0), d.get('y', 0.0)
        self.z, self.w = d.get('z', 0.0), d.get('w', 1.0)


def _parse(data: dict) -> Dict[str, Location]:
    locations = {}
    for name, entry in (data.get('positions') or {}).items():
        position = entry['pose']['position']
        yaw = quaternion_to_yaw(_Quaternion(entry['pose']['orientation']))
        frame_id = (entry.get('header') or {}).get('frame_id') or 'map'
        locations[str(name)] = Location(frame_id, Pose2D(position['x'], position['y'], yaw))
    return locations
