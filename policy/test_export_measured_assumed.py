"""Behavioral contract for explicitly assumed ground-entry profile."""
import hashlib,json,unittest
from pathlib import Path
import numpy as np
import yaml
from export_measured_assumed import export_assumed_profile
from demo_contract import check_bundle
from ground_carry_clearance import pedestal_geometry,safe_carry_entry
from export_hardware_loading_profile import sample_poses,post_place_exit_indices

class AssumedMeasuredProfileTest(unittest.TestCase):
 def test_profile_preserves_checked_inside_paths_and_honest_provenance(self):
  root=Path(__file__).resolve().parent
  bundle=root/'demo_data/measured8_spaced_inside_full'
  template=root.parent/'deployment_ws/config/loading_cycle.site.template.json'
  data=export_assumed_profile(bundle,template)
  plan=check_bundle(bundle,allow_partial=True)
  self.assertEqual(len(data['cycles']),8)
  self.assertFalse(data['assumptions']['ground_entry_simulation_validated'])
  self.assertTrue(data['assumptions']['ground_entry_assumed_by_operator'])
  self.assertTrue(data['assumptions']['inside_and_reverse_simulation_validated'])
  self.assertFalse(data['assumptions']['hardware_motion_validated'])
  self.assertEqual(data['assumptions']['source_sha256']['trajectory.npz'],hashlib.sha256((bundle/'trajectory.npz').read_bytes()).hexdigest())
  pedestal=pedestal_geometry(yaml.safe_load((bundle/'scene_tf.yaml').read_text(encoding='utf8')))
  with np.load(bundle/'trajectory.npz',allow_pickle=False) as trace:
   for source,cycle in zip(plan['sequence'],data['cycles']):
    self.assertEqual(cycle['box_label'],source['label'])
    self.assertEqual(cycle['expected_demo_input_dims_m'],source['input_dims_m'])
    self.assertEqual(cycle['place_target']['position_m'],source['position_world_m'])
    ids=np.flatnonzero((trace['box_id']==source['id'])&(trace['stage']=='CARRY'))
    entry=safe_carry_entry(ids,trace['box_pose'],source['input_dims_m'],pedestal)
    self.assertEqual(cycle['place_target']['via_box_poses'],sample_poses(trace['box_pose'][ids[ids>=entry]]))
    self.assertEqual(cycle['exit_tool_poses'],sample_poses(trace['tool_pose'][post_place_exit_indices(trace,source['id'])]))
