import copy
import json
from pathlib import Path
import unittest
from prepare_sides_first import sides_first


class SidesFirstTests(unittest.TestCase):
    def test_hardware_template_matches_checked_sides_first_plan(self):
        import hashlib
        root=Path(__file__).resolve().parents[1]
        profile=json.loads((root/'deployment_ws/config/loading_cycle.site.template.json').read_text(encoding='utf-8'))
        source=root/profile['assumptions']['source_bundle']
        plan=json.loads((source/'plan.json').read_text(encoding='utf-8'))
        expected=['red','silver','blue','bronzewhite','black','grey','pink','smallgrey']
        self.assertEqual([cycle['box_label'] for cycle in profile['cycles']],expected)
        self.assertEqual([row['label'] for row in plan['sequence']],expected)
        self.assertEqual(profile['assumptions']['source_sha256']['plan.json'],
                         hashlib.sha256((source/'plan.json').read_bytes()).hexdigest())
        for cycle,row in zip(profile['cycles'],plan['sequence']):
            self.assertEqual(cycle['expected_demo_input_dims_m'],row['input_dims_m'])
            self.assertEqual(cycle['place_target']['position_m'],row['position_world_m'])

    def test_reorders_real_inventory_without_changing_physical_targets(self):
        path=Path(__file__).parent/'demo_data/measured8_spaced_inside_full/plan.json'
        original=json.loads(path.read_text(encoding='utf-8'))['sequence']
        unchanged=copy.deepcopy(original)
        rows=sides_first(original)
        self.assertEqual([row['label'] for row in rows],
            ['red','silver','blue','bronzewhite','black','grey','pink','smallgrey'])
        self.assertEqual([row['step'] for row in rows],list(range(1,9)))
        self.assertEqual(original,unchanged)
        for row in rows:
            before=next(item for item in original if item['id']==row['id'])
            for field in ('input_index','input_dims_m','position_world_m','orientation_world_xyzw','stack_layer'):
                self.assertEqual(row[field],before[field])


if __name__=='__main__':unittest.main()
