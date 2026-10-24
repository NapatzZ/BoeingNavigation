"""
Which saved map is the one map_server is serving.

map_server reads its `yaml_filename` parameter once, at start; a later
`load_map` call does not update it. So the parameter is checked against the
map on /map (size, resolution, origin) before it is trusted.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import List, Optional

import yaml

from ..map_editor import read_pgm


@dataclass(frozen=True)
class MapShape:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def matches(self, other: "MapShape") -> bool:
        return (
            (self.width, self.height) == (other.width, other.height)
            and math.isclose(self.resolution, other.resolution, rel_tol=1e-6)
            and math.isclose(self.origin_x, other.origin_x, abs_tol=1e-3)
            and math.isclose(self.origin_y, other.origin_y, abs_tol=1e-3)
        )


def shape_of_map_yaml(map_yaml: Path) -> Optional[MapShape]:
    """The saved map's shape; None when its yaml or image cannot be read."""
    try:
        meta = yaml.safe_load(Path(map_yaml).read_text())
        width, height, _ = read_pgm(Path(map_yaml).parent / meta["image"])
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        return MapShape(width, height, float(meta["resolution"]), origin[0], origin[1])
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        return None


def find_running_map(
    live: MapShape, reported: Path, maps_root: Optional[Path]
) -> Optional[Path]:
    """
    The saved map that `live` (the map on /map) is, or None when unclear.

    `reported` is map_server's `yaml_filename`; it wins while it still matches.
    After a runtime swap it is stale, so the one other map that matches is used.
    Zero or several matches return None: no guessing between maps.
    """
    reported_shape = shape_of_map_yaml(reported)
    if reported_shape is None or reported_shape.matches(live):
        return reported
    if maps_root is None:
        return None
    candidates: List[Path] = [
        path
        for path in sorted(Path(maps_root).rglob("*.yaml"))
        if (shape := shape_of_map_yaml(path)) is not None and shape.matches(live)
    ]
    return candidates[0] if len(candidates) == 1 else None
