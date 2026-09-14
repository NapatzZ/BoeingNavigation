#!/usr/bin/env python3
"""Small pure-math geometry helpers shared across navigation nodes.

These were previously copy-pasted in nav_to_pose.py and save_location.py.
Everything here is pure (no ROS state), so it is trivially unit-testable and
safe to reuse.
"""
import math


def normalize_angle(angle: float) -> float:
    """Wrap an angle (radians) into the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """Extract the yaw (radians) from a quaternion-like object (.w/.x/.y/.z)."""
    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)


def yaw_to_quaternion(yaw: float):
    """Convert a yaw (radians) to the (z, w) components of a planar quaternion.

    x and y are always 0 for a rotation about the z axis, so only z and w are
    returned.
    """
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


# Named approach sides, in radians. Same eight names as save_location.py's
# ZONES so a side detected when saving a location can be fed straight back into
# a navigation request.
#
# These angles are bearings measured FROM the target outward. Interpretation
# depends on the reference yaw passed to approach_pose_for_side(): pass 0.0 and
# they are map-frame directions (FRONT = map +X); pass the target's own yaw and
# they become object-relative.
SIDE_OFFSETS = {
    'FRONT': 0.0,
    'FRONT_LEFT': math.pi / 4.0,
    'LEFT': math.pi / 2.0,
    'BACK_LEFT': 3.0 * math.pi / 4.0,
    'BACK': math.pi,
    'BACK_RIGHT': -3.0 * math.pi / 4.0,
    'RIGHT': -math.pi / 2.0,
    'FRONT_RIGHT': -math.pi / 4.0,
}

# Request values that mean "no side requested; use the legacy robot->target
# approach". Compared case-insensitively after stripping.
AUTO_SIDES = ('', 'AUTO', 'NONE')


def approach_pose_for_side(target_x, target_y, reference_yaw, side, standoff):
    """Standoff pose on `side` of a target, oriented to look back at it.

    `side` is a key of SIDE_OFFSETS. `reference_yaw` is the frame the side is
    measured in (0.0 for map-relative). `standoff` is the distance from the
    target origin to the returned point.

    Returns (x, y, yaw). Raises KeyError for an unknown side.
    """
    bearing = reference_yaw + SIDE_OFFSETS[side]
    x = target_x + standoff * math.cos(bearing)
    y = target_y + standoff * math.sin(bearing)
    # Stand on `side` looking back at the target: the inverse of the outward
    # bearing.
    return x, y, normalize_angle(bearing + math.pi)
