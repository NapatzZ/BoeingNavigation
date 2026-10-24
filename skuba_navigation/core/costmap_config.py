"""
Pure costmap/controller reconfiguration logic for costmap_reconfig_server.py.

No rclpy import. Everything here is YAML/dict manipulation and validation that
does not need a live parameter-service call.
"""

import time

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
    "max_obstacle_height": "voxel_layer.max_obstacle_height",
    "obstacle_max_range": "voxel_layer.scan.obstacle_max_range",
    "obstacle_min_range": "voxel_layer.scan.obstacle_min_range",
    "raytrace_max_range": "voxel_layer.scan.raytrace_max_range",
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


def update_yaml_config(config_path, costmap_type, param_name, new_value):
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        old_value = None
        costmap_key = f"{costmap_type}_costmap"

        if costmap_key not in config:
            return {
                "success": False,
                "error": f"Costmap '{costmap_type}' not found in config",
            }

        costmap_config = config[costmap_key][costmap_key]["ros__parameters"]

        if param_name in costmap_config:
            old_value = costmap_config[param_name]
            costmap_config[param_name] = new_value
        else:
            found = False
            for section_name, section_config in costmap_config.items():
                if isinstance(section_config, dict) and param_name in section_config:
                    old_value = section_config[param_name]
                    section_config[param_name] = new_value
                    found = True
                    break
                if isinstance(section_config, dict):
                    for nested_name, nested_config in section_config.items():
                        if (
                            isinstance(nested_config, dict)
                            and param_name in nested_config
                        ):
                            old_value = nested_config[param_name]
                            nested_config[param_name] = new_value
                            found = True
                            break
                    if found:
                        break
            if not found:
                return {
                    "success": False,
                    "error": f"Parameter '{param_name}' not found in {costmap_type}_costmap",
                }

        with open(config_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)

        return {"success": True, "old_value": old_value}

    except Exception as e:
        return {"success": False, "error": f"Failed to update YAML: {str(e)}"}


def create_backup_config(config_path):
    try:
        timestamp = int(time.time())
        backup_file = config_path.replace(".yaml", f"_backup_{timestamp}.yaml")
        with open(config_path, "r") as src:
            content = src.read()
        with open(backup_file, "w") as dst:
            dst.write(content)
        return {"success": True, "backup_file": backup_file}
    except Exception as e:
        return {"success": False, "error": str(e)}


def get_original_config() -> dict:
    return {
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "resolution": 0.05,
                    "width": 3,
                    "height": 3,
                    "update_frequency": 5.0,
                    "publish_frequency": 2.0,
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 2.0,
                        "inflation_radius": 0.3,
                    },
                    "voxel_layer": {
                        "plugin": "nav2_costmap_2d::VoxelLayer",
                        "enabled": True,
                        "max_obstacle_height": 2.0,
                        "scan": {
                            "obstacle_max_range": 2.5,
                            "obstacle_min_range": 0.3,
                            "raytrace_max_range": 3.0,
                        },
                    },
                }
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "resolution": 0.05,
                    "update_frequency": 1.0,
                    "publish_frequency": 1.0,
                    "track_unknown_space": True,
                    "inflation_layer": {
                        "plugin": "nav2_costmap_2d::InflationLayer",
                        "cost_scaling_factor": 3.0,
                        "inflation_radius": 0.35,
                    },
                    "obstacle_layer": {
                        "plugin": "nav2_costmap_2d::ObstacleLayer",
                        "enabled": True,
                        "scan": {
                            "obstacle_max_range": 2.5,
                            "obstacle_min_range": 0.3,
                            "raytrace_max_range": 3.0,
                        },
                    },
                }
            }
        },
    }


def write_config(config_path, config: dict) -> None:
    with open(config_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, indent=2)
