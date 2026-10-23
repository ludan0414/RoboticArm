"""Shared YAML and runtime binding retain the existing perception defaults."""
import tempfile
import unittest
from pathlib import Path
import yaml
from luggage_planning.pick_runtime_config import driver_defaults
from luggage_perception.runtime_tuning import configured_dataclass
from luggage_perception.instance_depth_component import ComponentConfig

class RuntimeConfigTests(unittest.TestCase):
    def test_site_yaml_preserves_component_acceptance_defaults(self):
        path = Path(__file__).resolve().parents[2] / "luggage_description/config/pick_runtime.site.yaml"
        config = yaml.safe_load(path.read_text())
        values = config["luggage_detector"]["ros__parameters"]
        def declare(name, value): values.setdefault(name,value)
        result = configured_dataclass(ComponentConfig,"component",declare,values.__getitem__)
        self.assertEqual(result, ComponentConfig())
        self.assertEqual(values["support_mode"], "auto")
        self.assertEqual(driver_defaults(path)["vacuum_timeout"], 45)
    def test_binding_reads_provided_parameters(self):
        values = {"component.erode_bbox_px":6}
        result = configured_dataclass(ComponentConfig,"component",lambda k,v:values.setdefault(k,v),values.__getitem__)
        self.assertEqual(result.erode_bbox_px, 6)
    def test_bad_timeouts_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"bad.yaml"
            for value in [-1, 0, True, "slow", float("nan")]:
                path.write_text(yaml.safe_dump({"hardware_pick_driver":{"ros__parameters":{"vacuum_timeout":value}}}))
                with self.assertRaises(ValueError): driver_defaults(path)
