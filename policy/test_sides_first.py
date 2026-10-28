import copy
import json
from pathlib import Path
import unittest
from prepare_sides_first import sides_first


class SidesFirstTests(unittest.TestCase):
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
