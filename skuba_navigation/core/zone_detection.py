"""Pure zone-accumulation + saved-position YAML logic for save_location.py.

No rclpy import.
"""
import math
import os
from collections import defaultdict

import numpy as np
import yaml

from .geometry_utils import SIDE_OFFSETS

ZONES = {
    "FRONT":       0.0,
    "FRONT_LEFT":  45.0,
    "LEFT":        90.0,
    "BACK_LEFT":   135.0,
    "BACK":        180.0,
    "BACK_RIGHT":  225.0,
    "RIGHT":       270.0,
    "FRONT_RIGHT": 315.0,
}
HALF_WIDTH_DEG = 25.0
OBSTACLE_RANGE = 1.0   # m


def accumulate_zone_hits(msg, zone_accumulator: dict) -> None:
    """Add this scan's per-zone obstacle-hit counts into `zone_accumulator`."""
    ranges = np.array(msg.ranges)
    ranges[~np.isfinite(ranges)] = np.inf

    num_beams = len(msg.ranges)
    # TF base_link→laser: yaw=π, roll=0 → θ_base = θ_laser + π
    angles = np.pi + (msg.angle_min + np.arange(num_beams) * msg.angle_increment)
    half_rad = math.radians(HALF_WIDTH_DEG)

    for side, center_deg in ZONES.items():
        center_rad = math.radians(center_deg)
        diff = (angles - center_rad + math.pi) % (2.0 * math.pi) - math.pi
        zone_idx = np.where(np.abs(diff) <= half_rad)[0]

        zone_ranges = ranges[zone_idx]
        hits = int(np.sum(
            (zone_ranges >= msg.range_min) & (zone_ranges <= OBSTACLE_RANGE)
        ))
        zone_accumulator[side] += hits


def best_zone(zone_accumulator: dict) -> str:
    """Pick the zone with the most accumulated hits, or "NONE" if all zero."""
    best_side = max(zone_accumulator, key=zone_accumulator.get)
    if zone_accumulator[best_side] == 0:
        return "NONE"
    return best_side


def new_zone_accumulator():
    return defaultdict(int, {side: 0 for side in ZONES})


def yaw_for_side(current_yaw: float, side: str) -> float:
    """Orient yaw toward the obstacle on `side` (side offset from robot yaw)."""
    return current_yaw + SIDE_OFFSETS.get(side, 0.0)


def save_location_to_yaml(yaml_path, location_name, side, translation, orientation) -> dict:
    """Merge one named location into the saved_pose.yaml file and write it.

    `translation`/`orientation` are (x, y, z) / (x, y, z, w) tuples.
    Returns the data dict that was written.
    """
    data = {"positions": {}}
    if yaml_path.exists():
        try:
            with open(yaml_path, 'r') as f:
                loaded = yaml.safe_load(f)
                if loaded and "positions" in loaded:
                    data = loaded
        except Exception:
            pass

    tx, ty, tz = translation
    ox, oy, oz, ow = orientation
    data["positions"][location_name] = {
        "header": {
            "frame_id": "map",
            "side": side,
        },
        "pose": {
            "position": {"x": tx, "y": ty, "z": tz},
            "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        },
    }

    os.makedirs(yaml_path.parent, exist_ok=True)
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False)

    return data
