"""Pure math / YAML logic for nav_to_pose.py. No rclpy import.

The retry/timeout control loops in nav_to_pose.py are inherently ROS I/O
sequencing (TF lookups + cmd_vel publishing inside `while rclpy.ok()` loops)
and stay in the node. This module holds only what can be computed or
file-read/written without a live node.
"""

import math

import yaml
from geometry_msgs.msg import PoseStamped

from .geometry_utils import normalize_angle, SIDE_OFFSETS, yaw_to_quaternion

# ── Rotation PD-controller tuning (see rotate_callback for rationale) ──
ROTATE_KP = 3.0
ROTATE_KD = 0.3
ROTATE_MAX_SPEED = 0.5
ROTATE_MIN_SPEED = 0.18
ROTATE_DECEL_ZONE = math.radians(5.0)

# Angular half-width of an approach side: sides are 45 deg apart, so 22.5
# deg either way is the widest a final pose can drift and still be
# unambiguously on the requested side.
SIDE_BEARING_TOLERANCE = math.radians(22.5)
# Radial slack on the standoff distance, matching the legacy AUTO check.
STANDOFF_TOLERANCE = 0.15
# Final heading error above which we spin in place to re-face the target.
FACING_TOLERANCE = math.radians(5.0)


def load_saved_positions(yaml_path) -> dict:
    """Parse saved_pose.yaml into a dict of name -> PoseStamped."""
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)
        positions = {}
        for name, pose_data in data.get("positions", {}).items():
            pose = PoseStamped()
            pose.header.frame_id = pose_data["header"]["frame_id"]
            pose.pose.position.x = pose_data["pose"]["position"]["x"]
            pose.pose.position.y = pose_data["pose"]["position"]["y"]
            pose.pose.position.z = pose_data["pose"]["position"]["z"]
            pose.pose.orientation.x = pose_data["pose"]["orientation"]["x"]
            pose.pose.orientation.y = pose_data["pose"]["orientation"]["y"]
            pose.pose.orientation.z = pose_data["pose"]["orientation"]["z"]
            pose.pose.orientation.w = pose_data["pose"]["orientation"]["w"]
            positions[name] = pose
        return positions


def merge_position(
    yaml_path, position_name, frame_id, translation, orientation
) -> dict:
    """Read saved_pose.yaml (or start empty) and merge in one named position.

    `translation`/`orientation` are (x, y, z) / (x, y, z, w) tuples. Does not
    write — caller decides when/whether to persist (e.g. after a write-
    permission check).
    """
    try:
        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file) or {"positions": {}}
    except Exception:
        data = {"positions": {}}

    tx, ty, tz = translation
    ox, oy, oz, ow = orientation
    data["positions"][position_name] = {
        "header": {"frame_id": frame_id},
        "pose": {
            "position": {"x": tx, "y": ty, "z": tz},
            "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        },
    }
    return data


def write_positions_yaml(yaml_path, data: dict) -> None:
    with open(yaml_path, "w") as file:
        yaml.safe_dump(data, file)


def distance_2d(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def compute_rotation_command(angle_diff, error_rate) -> float:
    """PD control law for in-place rotation. Returns signed angular speed.

    KP: stiff response - fast approach, strong braking.
    KD: damps overshoot by opposing the approach speed.
    Within DECEL_ZONE, force MIN_SPEED to avoid overshoot on the final
    micro-adjustment (floor friction otherwise stalls a smaller command).
    """
    control_speed = abs(angle_diff) * ROTATE_KP + error_rate * ROTATE_KD

    if abs(angle_diff) < ROTATE_DECEL_ZONE:
        speed = ROTATE_MIN_SPEED
    elif control_speed > ROTATE_MAX_SPEED:
        speed = ROTATE_MAX_SPEED
    elif control_speed < ROTATE_MIN_SPEED:
        speed = ROTATE_MIN_SPEED
    else:
        speed = control_speed

    return speed if angle_diff > 0 else -speed


def path_is_plannable(path) -> bool:
    """Interpret a nav2_simple_commander getPath() result."""
    if path is None:
        return False
    return len(path.poses) > 0


def make_pose(x, y, yaw, stamp, frame_id="map") -> PoseStamped:
    """Build a PoseStamped in `frame_id` from a planar (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def verify_directed_standoff(
    final_x, final_y, target_x, target_y, requested_side, standoff
):
    """Check the final pose against a directed (named-side) standoff goal.

    Returns (success: bool, final_distance, message).
    """
    final_distance = distance_2d(final_x, final_y, target_x, target_y)
    expected_bearing = SIDE_OFFSETS[requested_side]
    actual_bearing = math.atan2(final_y - target_y, final_x - target_x)
    bearing_error = abs(normalize_angle(actual_bearing - expected_bearing))

    distance_ok = abs(final_distance - standoff) <= STANDOFF_TOLERANCE
    side_ok = bearing_error <= SIDE_BEARING_TOLERANCE

    log_msg = (
        f"Final: distance {final_distance:.3f}m (want {standoff:.3f}m), "
        f"bearing off by {math.degrees(bearing_error):.1f}° from side {requested_side}"
    )

    if distance_ok and side_ok:
        return True, final_distance, log_msg
    wrong = "distance" if not distance_ok else "side"
    return False, final_distance, f"Ended up with the wrong {wrong}. {log_msg}"


def verify_auto_standoff(final_x, final_y, target_x, target_y, stop_distance):
    """Check the final pose against the legacy AUTO (undirected) standoff goal.

    Returns (success: bool, final_distance, message).
    """
    final_distance = distance_2d(final_x, final_y, target_x, target_y)
    within_tolerance = final_distance <= (stop_distance + STANDOFF_TOLERANCE)
    log_msg = (
        f"Final distance to target: {final_distance:.3f}m "
        f"(goal: {stop_distance:.3f}m, within_tolerance: {within_tolerance})"
    )
    return within_tolerance, final_distance, log_msg
