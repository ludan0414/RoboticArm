"""Bind component config fields using injected parameter accessors; no ROS."""
from dataclasses import fields


def configured_dataclass(config_type, prefix, declare, read):
    default = config_type()
    values = {}
    for field in fields(default):
        name = prefix + "." + field.name
        declare(name, getattr(default, field.name))
        values[field.name] = read(name)
    return config_type(**values)
