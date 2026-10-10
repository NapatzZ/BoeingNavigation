"""
Where to park next to a target frame, and whether we parked correctly.

Two modes, chosen by `side`:

  AUTO (None)  - stop `stop_distance` short of the target along the line the
                 robot already stands on; which side it ends up on depends on
                 where it started.
  named side   - park on that MAP-RELATIVE side of the target (FRONT = map +X,
                 LEFT = map +Y, ...) at object_radius + stop_distance, facing
                 it. Object TFs carry no usable heading, hence map-relative.
"""

import math
from typing import Optional, Tuple

from ..geometry_utils import (
    approach_pose_for_side,
    AUTO_SIDES,
    normalize_angle,
    SIDE_OFFSETS,
)
from .types import Pose2D


def parse_side(side: str) -> Optional[str]:
    """Normalise an approach_side string. None = AUTO. Raises ValueError if unknown."""
    name = side.strip().upper()
    if name in AUTO_SIDES:
        return None
    if name not in SIDE_OFFSETS:
        raise ValueError(
            f"Unknown approach_side '{side}'. "
            f"Valid: {', '.join(sorted(SIDE_OFFSETS))}, or AUTO."
        )
    return name


def standoff_goal(
    robot: Pose2D,
    target_x: float,
    target_y: float,
    stop_distance: float,
    side: Optional[str],
    object_radius: float,
    standoff_tolerance: float,
    facing_tolerance: float,
) -> Optional[Pose2D]:
    """Goal pose for Nav2, or None if the robot is already parked correctly."""
    if side is not None:
        x, y, yaw = approach_pose_for_side(
            target_x, target_y, 0.0, side, object_radius + stop_distance
        )
        # "Already there" is judged against the standoff POSE: the robot can be
        # inside stop_distance and still on entirely the wrong side.
        if (
            robot.distance_to(x, y) <= standoff_tolerance
            and abs(normalize_angle(yaw - robot.yaw)) <= facing_tolerance
        ):
            return None
        return Pose2D(x, y, yaw)

    distance = robot.distance_to(target_x, target_y)
    if distance <= stop_distance:
        return None
    heading = robot.bearing_to(target_x, target_y)
    travel = distance - stop_distance
    return Pose2D(
        robot.x + travel * math.cos(heading),
        robot.y + travel * math.sin(heading),
        heading,
    )


def verify_standoff(
    robot: Pose2D,
    target_x: float,
    target_y: float,
    stop_distance: float,
    side: Optional[str],
    object_radius: float,
    standoff_tolerance: float,
    side_bearing_tolerance: float,
) -> Tuple[bool, float, str]:
    """Check the final pose. Returns (ok, distance_to_target, explanation)."""
    distance = robot.distance_to(target_x, target_y)

    if side is None:
        ok = distance <= stop_distance + standoff_tolerance
        return (
            ok,
            distance,
            (
                f"Final distance to target {distance:.3f}m "
                f"(want <= {stop_distance:.3f}m + {standoff_tolerance:.2f}m)."
            ),
        )

    standoff = object_radius + stop_distance
    bearing = math.atan2(robot.y - target_y, robot.x - target_x)
    bearing_error = abs(normalize_angle(bearing - SIDE_OFFSETS[side]))
    distance_ok = abs(distance - standoff) <= standoff_tolerance
    side_ok = bearing_error <= side_bearing_tolerance
    detail = (
        f"distance {distance:.3f}m (want {standoff:.3f}m), bearing off by "
        f"{math.degrees(bearing_error):.1f} deg from side {side}."
    )
    if distance_ok and side_ok:
        return True, distance, f"Parked on side {side}: {detail}"
    wrong = "distance" if not distance_ok else "side"
    return False, distance, f"Ended up with the wrong {wrong}: {detail}"
