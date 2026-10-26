"""
Pure costmap/controller reconfiguration logic for costmap_reconfig_server.py.

No rclpy import. Validation, name mapping and the memory of replaced values;
the node only makes the parameter-service calls. Changes go to the running
costmaps only; params/rcj_param.yaml stays the one source of defaults.
"""

import math

import yaml
from rcl_interfaces.msg import ParameterType

# Parameter mapping: simple name -> full Nav2 parameter path. No robot_radius:
# the costmaps use a polygon `footprint` (params/*.yaml) that
# skuba_manipulation's arm_footprint owns at runtime.
PARAM_MAPPING = {
    "resolution": "resolution",
    "width": "width",
    "height": "height",
    "update_frequency": "update_frequency",
    "publish_frequency": "publish_frequency",
    "inflation_radius": "inflation_layer.inflation_radius",
    "cost_scaling_factor": "inflation_layer.cost_scaling_factor",
    "max_obstacle_height": "obstacle_layer.scan.max_obstacle_height",
    "obstacle_max_range": "obstacle_layer.scan.obstacle_max_range",
    "obstacle_min_range": "obstacle_layer.scan.obstacle_min_range",
    "raytrace_max_range": "obstacle_layer.scan.raytrace_max_range",
}

# Controller parameter registry — single source of truth (simple name ->
# full path on /controller_server, ParameterType, backward-mode-enable value,
# normal-mode-default value). A None enable/default means that param is
# reachable via controller_reconfig but is not touched by
# enable_backward_mode/disable_backward_mode.
# These are FollowPath (DWB / RPP) parameters — adjust plugin prefix to match
# your config. Common prefixes: "FollowPath" (RPP), "FollowPath.critics" (DWB).
_CONTROLLER_PARAMS = {
    # name: (full path, type, backward value, normal value)
    "allow_reversing": (
        "FollowPath.allow_reversing",
        ParameterType.PARAMETER_BOOL,
        True,
        False,
    ),
    "min_vel_x": ("FollowPath.min_vel_x", ParameterType.PARAMETER_DOUBLE, -0.7, 0.0),
    "max_vel_x": ("FollowPath.max_vel_x", ParameterType.PARAMETER_DOUBLE, 0.0, 0.5),
    "max_vel_x_backwards": (
        "FollowPath.max_vel_x_backwards",
        ParameterType.PARAMETER_DOUBLE,
        0.8,
        0.3,
    ),
    "max_vel_theta": (
        "FollowPath.max_vel_theta",
        ParameterType.PARAMETER_DOUBLE,
        None,
        None,
    ),
    "min_speed_xy": (
        "FollowPath.min_speed_xy",
        ParameterType.PARAMETER_DOUBLE,
        None,
        None,
    ),
    "max_speed_xy": (
        "FollowPath.max_speed_xy",
        ParameterType.PARAMETER_DOUBLE,
        None,
        None,
    ),
}

CONTROLLER_PARAM_MAPPING = {
    name: path for name, (path, _, _, _) in _CONTROLLER_PARAMS.items()
}
CONTROLLER_PARAM_TYPES = {
    name: ptype for name, (_, ptype, _, _) in _CONTROLLER_PARAMS.items()
}
BACKWARD_MODE_ENABLE_CHANGES = [
    (name, enable_val, ptype)
    for name, (_, ptype, enable_val, _) in _CONTROLLER_PARAMS.items()
    if enable_val is not None
]
BACKWARD_MODE_DEFAULTS = {
    name: (default_val, ptype)
    for name, (_, ptype, _, default_val) in _CONTROLLER_PARAMS.items()
    if default_val is not None
}


def load_validation_rules() -> dict:
    return {
        "resolution": {"min": 0.01, "max": 0.1},
        "width": {"min": 2, "max": 10},
        "height": {"min": 2, "max": 10},
        "update_frequency": {"min": 1.0, "max": 20.0},
        "publish_frequency": {"min": 0.5, "max": 10.0},
        "inflation_radius": {"min": 0.1, "max": 1.0},
        "cost_scaling_factor": {"min": 1.0, "max": 10.0},
        "max_obstacle_height": {"min": 0.5, "max": 5.0},
        "obstacle_max_range": {"min": 1.0, "max": 5.0},
        "obstacle_min_range": {"min": 0.1, "max": 1.0},
        "raytrace_max_range": {"min": 1.0, "max": 8.0},
    }


def convert_value(value_str, param_type):
    """Convert string value to the correct Python type for a ParameterType."""
    try:
        if param_type == ParameterType.PARAMETER_BOOL:
            if isinstance(value_str, bool):
                return value_str
            return value_str.strip().lower() in ("true", "1", "yes")
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            return float(value_str)
        elif param_type == ParameterType.PARAMETER_INTEGER:
            return int(value_str)
        else:
            return str(value_str)
    except Exception:
        return None


def validate_parameter(param_name, value_str, validation_rules):
    try:
        converted_value = float(value_str)
        if param_name in validation_rules:
            rule = validation_rules[param_name]
            if "min" in rule and converted_value < rule["min"]:
                return {
                    "valid": False,
                    "error": f"Value {converted_value} below minimum {rule['min']}",
                }
            if "max" in rule and converted_value > rule["max"]:
                return {
                    "valid": False,
                    "error": f"Value {converted_value} above maximum {rule['max']}",
                }
        return {"valid": True, "converted_value": converted_value}
    except ValueError:
        return {"valid": False, "error": f"Cannot convert '{value_str}' to number"}


def footprint_radius(footprint, robot_radius):
    """
    Return the robot's outer radius (m) from a costmap's footprint parameters.

    footprint is the costmap's polygon string, e.g. "[[0.3, 0.0], ...]"; with
    "[]" the costmap uses robot_radius instead, as Nav2 does.
    """
    try:
        points = yaml.safe_load(footprint) if footprint else []
    except yaml.YAMLError:
        points = None
    if not points:
        return robot_radius
    return max(math.hypot(float(x), float(y)) for x, y in points)


def check_inflation_radius(value, robot_radius):
    """
    Return an error if value is smaller than the robot's radius, else None.

    A smaller inflation radius cuts the no-go band around walls short, so the
    change would do nothing useful.
    """
    if value < robot_radius:
        return (
            f"inflation_radius {value} is below the robot's footprint radius "
            f"{robot_radius:.2f} m"
        )
    return None


class OriginalValues:
    """
    Remember the value each parameter had before its first change.

    Keys are (costmap, nav2 name). Only the first change of a key is kept, so a
    reset goes back to the value Nav2 started with, not to an earlier tweak.
    """

    def __init__(self):
        self._values = {}

    def remember(self, key, value):
        self._values.setdefault(key, value)

    def items(self):
        return list(self._values.items())

    def forget(self, key):
        self._values.pop(key, None)
