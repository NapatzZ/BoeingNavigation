"""Unit tests for costmap_reconfig_server.py's pure validation/YAML/dict logic."""

import pytest
import yaml
from rcl_interfaces.msg import ParameterType

from skuba_navigation.core import costmap_config as cc


def test_load_validation_rules_covers_every_costmap_param():
    rules = cc.load_validation_rules()
    assert set(rules) == set(cc.PARAM_MAPPING)
    assert rules["robot_radius"] == {"min": 0.05, "max": 0.5}


def test_validate_parameter_within_range():
    rules = cc.load_validation_rules()
    result = cc.validate_parameter("inflation_radius", "0.5", rules)
    assert result == {"valid": True, "converted_value": 0.5}


@pytest.mark.parametrize("value", ["0.0", "5.0"])
def test_validate_parameter_outside_range_is_rejected(value):
    rules = cc.load_validation_rules()
    result = cc.validate_parameter("inflation_radius", value, rules)
    assert result["valid"] is False
    assert "error" in result


def test_validate_parameter_non_numeric_is_rejected():
    rules = cc.load_validation_rules()
    result = cc.validate_parameter("inflation_radius", "not-a-number", rules)
    assert result["valid"] is False


def test_convert_value_by_type():
    assert cc.convert_value("true", ParameterType.PARAMETER_BOOL) is True
    assert cc.convert_value("0", ParameterType.PARAMETER_BOOL) is False
    assert cc.convert_value("1.5", ParameterType.PARAMETER_DOUBLE) == pytest.approx(1.5)
    assert cc.convert_value("3", ParameterType.PARAMETER_INTEGER) == 3
    assert cc.convert_value("hello", ParameterType.PARAMETER_STRING) == "hello"


def test_convert_value_returns_none_on_bad_input():
    assert cc.convert_value("not-a-number", ParameterType.PARAMETER_DOUBLE) is None


def _write_sample_config(path):
    path.write_text(
        yaml.dump(
            {
                "local_costmap": {
                    "local_costmap": {
                        "ros__parameters": {
                            "robot_radius": 0.1,
                            "inflation_layer": {"cost_scaling_factor": 2.0},
                        }
                    }
                }
            }
        )
    )


def test_update_yaml_config_top_level_param(tmp_path):
    config_path = tmp_path / "costmap_dynamic_config.yaml"
    _write_sample_config(config_path)

    result = cc.update_yaml_config(str(config_path), "local", "robot_radius", 0.2)
    assert result == {"success": True, "old_value": 0.1}

    on_disk = yaml.safe_load(config_path.read_text())
    assert (
        on_disk["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]
        == 0.2
    )


def test_update_yaml_config_nested_param(tmp_path):
    config_path = tmp_path / "costmap_dynamic_config.yaml"
    _write_sample_config(config_path)

    result = cc.update_yaml_config(
        str(config_path), "local", "cost_scaling_factor", 4.0
    )
    assert result == {"success": True, "old_value": 2.0}

    on_disk = yaml.safe_load(config_path.read_text())
    nested = on_disk["local_costmap"]["local_costmap"]["ros__parameters"][
        "inflation_layer"
    ]
    assert nested["cost_scaling_factor"] == 4.0


def test_update_yaml_config_unknown_param_fails(tmp_path):
    config_path = tmp_path / "costmap_dynamic_config.yaml"
    _write_sample_config(config_path)

    result = cc.update_yaml_config(str(config_path), "local", "does_not_exist", 1.0)
    assert result["success"] is False


def test_update_yaml_config_unknown_costmap_fails(tmp_path):
    config_path = tmp_path / "costmap_dynamic_config.yaml"
    _write_sample_config(config_path)

    result = cc.update_yaml_config(str(config_path), "nope", "robot_radius", 1.0)
    assert result["success"] is False


def test_create_backup_config_copies_content(tmp_path):
    config_path = tmp_path / "costmap_dynamic_config.yaml"
    config_path.write_text("robot_radius: 0.1\n")

    result = cc.create_backup_config(str(config_path))
    assert result["success"] is True
    assert result["backup_file"] != str(config_path)
    with open(result["backup_file"]) as f:
        assert f.read() == "robot_radius: 0.1\n"


def test_controller_param_tables_stay_consistent_after_dedup():
    """Regression test for collapsing 4 hand-kept dicts/lists into one
    _CONTROLLER_PARAMS registry - every derived table must still agree."""
    assert set(cc.CONTROLLER_PARAM_MAPPING) == set(cc.CONTROLLER_PARAM_TYPES)

    enable_by_name = {
        name: (value, ptype) for name, value, ptype in cc.BACKWARD_MODE_ENABLE_CHANGES
    }
    assert enable_by_name == {
        "allow_reversing": (True, ParameterType.PARAMETER_BOOL),
        "max_vel_x": (0.0, ParameterType.PARAMETER_DOUBLE),
        "min_vel_x": (-0.7, ParameterType.PARAMETER_DOUBLE),
        "max_vel_x_backwards": (0.8, ParameterType.PARAMETER_DOUBLE),
    }
    assert cc.BACKWARD_MODE_DEFAULTS == {
        "allow_reversing": (False, ParameterType.PARAMETER_BOOL),
        "max_vel_x": (0.5, ParameterType.PARAMETER_DOUBLE),
        "min_vel_x": (0.0, ParameterType.PARAMETER_DOUBLE),
        "max_vel_x_backwards": (0.3, ParameterType.PARAMETER_DOUBLE),
    }
    for name in enable_by_name:
        assert name in cc.CONTROLLER_PARAM_MAPPING
