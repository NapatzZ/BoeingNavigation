"""Unit tests for costmap_reconfig_server.py's pure validation and memory logic."""

from pathlib import Path

import pytest
import yaml
from rcl_interfaces.msg import ParameterType

from skuba_navigation.core import costmap_config as cc


def test_load_validation_rules_covers_every_costmap_param():
    rules = cc.load_validation_rules()
    assert set(rules) == set(cc.PARAM_MAPPING)
    assert "robot_radius" not in rules  # the costmaps use a polygon footprint


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


@pytest.mark.parametrize("value", ["0.25", "0.29"])
def test_inflation_radius_below_footprint_radius_is_rejected(value):
    # Below the 0.30 m footprint radius the no-go band is cut short.
    rules = cc.load_validation_rules()
    assert cc.validate_parameter("inflation_radius", value, rules)["valid"] is False


def test_inflation_radius_at_footprint_radius_is_accepted():
    rules = cc.load_validation_rules()
    assert cc.validate_parameter("inflation_radius", "0.30", rules)["valid"] is True


def test_every_mapped_param_exists_on_the_running_local_costmap():
    rcj = Path(__file__).resolve().parents[1] / "params" / "rcj_param.yaml"
    params = yaml.safe_load(rcj.read_text())["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]
    for path in cc.PARAM_MAPPING.values():
        node = params
        for part in path.split("."):
            assert part in node, f"{path} is not in rcj_param.yaml"
            node = node[part]


def test_original_values_keep_the_first_value_only():
    originals = cc.OriginalValues()
    originals.remember(("local", "inflation_layer.inflation_radius"), 0.40)
    originals.remember(("local", "inflation_layer.inflation_radius"), 0.35)
    assert originals.items() == [(("local", "inflation_layer.inflation_radius"), 0.40)]


def test_original_values_forget_after_restore():
    originals = cc.OriginalValues()
    originals.remember(("global", "inflation_layer.inflation_radius"), 0.40)
    originals.forget(("global", "inflation_layer.inflation_radius"))
    originals.forget(("global", "never_set"))  # no error
    assert originals.items() == []


def test_controller_param_tables_stay_consistent_after_dedup():
    """
    Check every derived controller table still agrees.

    Regression test for collapsing 4 hand-kept dicts/lists into one
    _CONTROLLER_PARAMS registry.
    """
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
