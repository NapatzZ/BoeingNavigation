"""
File + coordinate logic for the map editor web tool. No rclpy import.

Maps follow the maps/<category>/<name>.{pgm,yaml} layout (see map_saver.py).
Only the nav2 occupancy image (.pgm) is edited; a SLAM Toolbox .posegraph next
to it is left as recorded, so re-serializing from SLAM would undo the edits.

Locations are the selected map's <name>.poses.yaml next to its yaml, which
navigation_server's LocationStore reads (re-read on change), in the schema
save_location writes:

    positions:
      <name>:
        header: {frame_id: map}
        pose:
          position: {x, y, z}
          orientation: {x, y, z, w}
"""

import copy
import shutil
import time
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import yaml

from .geometry_utils import quaternion_to_yaw, yaw_to_quaternion
from .navigation.location_store import poses_path_for_map, write_poses_file

# Trinary values map_saver_cli writes (mode: trinary, negate: 0).
FREE = 254
OCCUPIED = 0
UNKNOWN = 205

_WHITESPACE = b" \t\n\r\v\f"


# ── PGM ──────────────────────────────────────────────────────────────────


def _pgm_tokens(data, count, pos=0):
    """Read `count` whitespace-separated header tokens, skipping # comments."""
    tokens = []
    while len(tokens) < count:
        while pos < len(data) and data[pos] in _WHITESPACE:
            pos += 1
        if data[pos] == ord("#"):
            pos = data.index(b"\n", pos) + 1
            continue
        start = pos
        while pos < len(data) and data[pos] not in _WHITESPACE:
            pos += 1
        tokens.append(data[start:pos])
    return tokens, pos + 1  # exactly one whitespace byte before the raster


def read_pgm(path):
    """Return (width, height, pixels: bytearray) of an 8-bit PGM, binary (P5) or ASCII (P2)."""
    data = Path(path).read_bytes()
    (magic, width, height, maxval), pos = _pgm_tokens(data, 4)
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"{path}: only P5 or P2 PGM is supported")
    if int(maxval) > 255:
        raise ValueError(f"{path}: only 8-bit PGM is supported")
    width, height = int(width), int(height)
    if magic == b"P2":
        pixels = bytearray(int(v) for v in data[pos:].split()[: width * height])
    else:
        end = pos + width * height
        pixels = bytearray(data[pos:end])
    if len(pixels) != width * height:
        raise ValueError(f"{path}: truncated raster")
    return width, height, pixels


def write_pgm(path, width, height, pixels):
    """Write a binary (P5) 8-bit PGM."""
    if len(pixels) != width * height:
        raise ValueError("pixel count does not match width*height")
    Path(path).write_bytes(b"P5\n%d %d\n255\n" % (width, height) + bytes(pixels))


# ── map files ────────────────────────────────────────────────────────────


def list_maps(maps_root):
    """Every <category>/<name>.yaml (or <name>.yaml) map under maps_root."""
    root = Path(maps_root)
    found = []
    for yaml_path in sorted(root.rglob("*.yaml")):
        try:
            meta = yaml.safe_load(yaml_path.read_text())
        except (OSError, yaml.YAMLError):
            continue
        if isinstance(meta, dict) and "image" in meta and "resolution" in meta:
            found.append(str(yaml_path.relative_to(root).with_suffix("")))
    return found


def resolve_map(maps_root, map_id):
    """
    maps_root/<map_id>.yaml, refusing anything that escapes maps_root.

    map_id comes from the browser, so `..` or an absolute path must not let
    a request read or overwrite files elsewhere on the robot.
    """
    rel = PurePosixPath(map_id)
    if rel.is_absolute() or ".." in rel.parts or not rel.parts:
        raise ValueError(f"bad map id: {map_id!r}")
    path = (Path(maps_root) / rel).with_suffix(".yaml")
    if Path(maps_root).resolve() not in path.resolve().parents:
        raise ValueError(f"bad map id: {map_id!r}")
    return path


def poses_path(maps_root, map_id):
    """The map's poses file, next to its yaml (same escape checks as resolve_map)."""
    return poses_path_for_map(resolve_map(maps_root, map_id))


def load_map(maps_root, map_id):
    """Return the map metadata + raster for the editor."""
    yaml_path = resolve_map(maps_root, map_id)
    meta = yaml.safe_load(yaml_path.read_text())
    image_path = yaml_path.parent / meta["image"]
    width, height, pixels = read_pgm(image_path)
    origin = list(meta.get("origin", [0.0, 0.0, 0.0]))
    return {
        "id": map_id,
        "image": meta["image"],
        "width": width,
        "height": height,
        "resolution": float(meta["resolution"]),
        "origin": [float(v) for v in origin],
        "pixels": pixels,
    }


def backup_once(path):
    """
    Copy path to <path>.bak the first time it is overwritten.

    Keeping only the first backup means repeated saves never replace the
    as-recorded original with an intermediate edit.
    """
    path = Path(path)
    bak = path.with_name(path.name + ".bak")
    if path.exists() and not bak.exists():
        shutil.copy2(path, bak)
    return bak


def save_map_pixels(maps_root, map_id, width, height, pixels):
    """Overwrite the map's .pgm with edited pixels (original kept as .bak)."""
    yaml_path = resolve_map(maps_root, map_id)
    meta = yaml.safe_load(yaml_path.read_text())
    image_path = yaml_path.parent / meta["image"]
    current_w, current_h, _ = read_pgm(image_path)
    if (width, height) != (current_w, current_h):
        raise ValueError("edited map size does not match the file on disk")
    backup_once(image_path)
    write_pgm(image_path, width, height, pixels)
    return str(image_path)


# ── pixel <-> world ──────────────────────────────────────────────────────


def pixel_to_world(col, row, height, resolution, origin):
    """Cell centre (col, row from the image top) -> map-frame (x, y)."""
    x = origin[0] + (col + 0.5) * resolution
    y = origin[1] + (height - row - 0.5) * resolution
    return x, y


def world_to_pixel(x, y, height, resolution, origin):
    """Map-frame (x, y) -> fractional (col, row) from the image top."""
    col = (x - origin[0]) / resolution - 0.5
    row = height - (y - origin[1]) / resolution - 0.5
    return col, row


# ── locations (<map>.poses.yaml) ─────────────────────────────────────────


def load_locations(yaml_path):
    """Return [{name, x, y, yaw}] from a poses file (empty if missing)."""
    try:
        data = yaml.safe_load(Path(yaml_path).read_text()) or {}
    except FileNotFoundError:
        return []
    locations = []
    for name, entry in (data.get("positions") or {}).items():
        pos = entry["pose"]["position"]
        q = SimpleNamespace(**entry["pose"]["orientation"])
        locations.append(
            {
                "name": str(name),
                "x": float(pos["x"]),
                "y": float(pos["y"]),
                "yaw": quaternion_to_yaw(q),
            }
        )
    return locations


def locations_to_yaml_data(locations, existing=None):
    """
    [{name, x, y, yaw}] -> the poses dict LocationStore parses.

    `existing` is the file's current dict. Locations kept in the editor keep
    every field the editor does not own (e.g. save_location's header.side);
    only the pose is replaced. Names missing from `locations` are dropped.
    """
    old = ((existing or {}).get("positions")) or {}
    positions = {}
    for loc in locations:
        name = str(loc["name"]).strip()
        if not name:
            raise ValueError("every location needs a name")
        if name in positions:
            raise ValueError(f"duplicate location name: {name}")
        z, w = yaw_to_quaternion(float(loc["yaw"]))
        entry = copy.deepcopy(old.get(name)) or {}
        entry.setdefault("header", {}).setdefault("frame_id", "map")
        entry["pose"] = {
            "position": {"x": float(loc["x"]), "y": float(loc["y"]), "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": z, "w": w},
        }
        positions[name] = entry
    data = dict(existing or {})
    data["positions"] = positions
    return data


def save_locations(yaml_path, locations):
    """Write `locations` to a poses file, timestamped backup first."""
    yaml_path = Path(yaml_path)
    existing = {}
    if yaml_path.exists():
        existing = yaml.safe_load(yaml_path.read_text()) or {}
        stamp = time.strftime("%Y%m%d_%H%M%S")
        shutil.copy2(yaml_path, yaml_path.with_name(f"{yaml_path.name}.{stamp}.bak"))
    data = locations_to_yaml_data(locations, existing)
    write_poses_file(yaml_path, data)
    return str(yaml_path)
