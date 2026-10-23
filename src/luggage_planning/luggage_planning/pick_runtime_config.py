"""Driver defaults from the same site YAML used by the ROS node launches."""
import yaml


def driver_defaults(path):
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    values = data.get("hardware_pick_driver", {}).get("ros__parameters", {})
    allowed = {"vacuum_timeout", "vacuum_state_max_age_sec"}
    if set(values) - allowed:
        raise ValueError("unknown driver runtime keys: %s" % sorted(set(values)-allowed))
    if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not 0 < v < float("inf") for v in values.values()):
        raise ValueError("driver timeouts must be positive finite numbers")
    return values
