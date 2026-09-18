"""Goal checks run in the goal callback, before a goal is accepted.

Each function returns None when the goal is acceptable, else the reason it is
rejected. A rejected goal never starts, so a behavior tree fails on the same
tick instead of after a timeout.
"""
import math
from typing import Optional

from .standoff import parse_side


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def check_go_to_pose(x: float, y: float, qx: float, qy: float, qz: float,
                     qw: float) -> Optional[str]:
    if not _finite(x, y, qx, qy, qz, qw):
        return 'pose contains NaN/inf.'
    if qx * qx + qy * qy + qz * qz + qw * qw < 1e-6:
        return 'pose orientation is a zero quaternion.'
    return None


def check_go_to_location(location: str, known: bool) -> Optional[str]:
    if not location.strip():
        return 'location is empty.'
    if not known:
        return f"location '{location.strip()}' is not in the locations file."
    return None


def check_go_to_frame(frame: str, stop_distance: float, approach_side: str,
                      object_radius: float) -> Optional[str]:
    if not frame.strip():
        return 'frame is empty.'
    if not _finite(stop_distance, object_radius) or stop_distance < 0 or object_radius < 0:
        return 'stop_distance and object_radius must be finite and >= 0.'
    try:
        parse_side(approach_side)
    except ValueError as e:
        return str(e)
    return None


def check_rotate(angle_deg: float, tolerance_deg: float) -> Optional[str]:
    if not _finite(angle_deg, tolerance_deg):
        return 'angle_deg / tolerance_deg must be finite.'
    return None


def check_drive(distance: float, speed: float) -> Optional[str]:
    if not _finite(distance, speed):
        return 'distance / speed must be finite.'
    return None


def check_nudge(frame: str, stop_distance: float) -> Optional[str]:
    if not frame.strip():
        return 'frame is empty.'
    if not _finite(stop_distance) or stop_distance < 0:
        return 'stop_distance must be finite and >= 0.'
    return None
