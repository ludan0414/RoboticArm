import unittest
import json
from pathlib import Path
from three_stack_layout import build_three_stacks, lateral_gap
from online_packing_real import Hull

class ThreeStackTest(unittest.TestCase):
 def test_distinct_inventory_supported_and_at_most_four_layers(self):
  inventory=json.loads((Path(__file__).parent/'demo_inputs/site_thirteen_measured.json').read_text(encoding='utf-8'))
  hull=Hull(1.49,1.97,1.48,1.535,.37,.057,1.383,.067,1.479,.53,-.745,-.985)
  result=build_three_stacks(inventory,hull)
  self.assertEqual(len(result),8)
  self.assertEqual(len({i for _,i,_ in result}),8)
  self.assertEqual([level for _,_,level in result],[1,2,3,1,2,1,2,3])
 def test_shifted_layout_keeps_five_centimetres_between_stacks(self):
  inventory=json.loads((Path(__file__).parent/'demo_inputs/site_thirteen_measured.json').read_text(encoding='utf-8'))
  hull=Hull(1.49,1.97,1.48,1.535,.37,.057,1.383,.067,1.479,.53,-.745,-.985)
  result=build_three_stacks(inventory,hull,((.60,.35),(.60,.84),(.60,1.32)))
  groups=(result[:3],result[3:5],result[5:])
  for left_index in range(3):
   for right_index in range(left_index+1,3):
    self.assertTrue(all(lateral_gap(a[0],b[0]) >= .05-1e-7
                        for a in groups[left_index] for b in groups[right_index]))
