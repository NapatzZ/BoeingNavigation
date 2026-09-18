"""Declare a config dataclass as ROS parameters `<prefix>.<field>`."""
import dataclasses
from typing import Type, TypeVar

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

T = TypeVar('T')
_SCALARS = (bool, int, float)


def declare_dataclass(node: Node, prefix: str, cls: Type[T]) -> T:
    """Declare every field of `cls` (default = field default) and build an instance.

    dynamic_typing lets a YAML `2` feed a float field (and vice versa); the
    value is then cast to the field's declared type.
    """
    values = {}
    for field in dataclasses.fields(cls):
        name = f'{prefix}.{field.name}'
        node.declare_parameter(name, field.default, ParameterDescriptor(dynamic_typing=True))
        value = node.get_parameter(name).value
        values[field.name] = field.type(value) if field.type in _SCALARS else value
    return cls(**values)
