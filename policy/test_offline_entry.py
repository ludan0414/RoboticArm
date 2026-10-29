import copy,json,tempfile,unittest,subprocess,sys,os
from pathlib import Path
from plan_boxes import load_inventory
from luggage_planning.offline_route_contract import validation_stamp,require_validation,PROFILE_TYPE


class OfflineInputTests(unittest.TestCase):
    def test_existing_thirteen_box_inventory_keeps_names_sizes_and_duplicates(self):
        result=load_inventory(Path(__file__).parent/'demo_inputs/site_thirteen_measured.json')
        self.assertEqual(len(result['boxes']),13)
        self.assertEqual(result['box_labels'][7:9],['blackwhite','blackbronze'])
        self.assertEqual(result['boxes'][7],[.75,.49,.29])
        self.assertEqual(result['boxes'][7],result['boxes'][8])

    def test_parallel_inventory_units_and_label_count(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'input.json'
            path.write_text(json.dumps(dict(units='cm',boxes=[[55,35,20]],box_labels=['a'])))
            self.assertEqual(load_inventory(path)['boxes'],[[.55,.35,.2]])
            for data in (dict(units='m',boxes=[[1,2,3]],box_labels=[]),
                         dict(units='mm',boxes=[[1,2,3]],box_labels=['a'])):
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):load_inventory(path)

    def test_offline_solver_imports_do_not_load_learned_policy(self):
        code="import plan_boxes,offline_frozen_planner,prepare_full_floor_demo,sys; assert 'packing_policy' not in sys.modules; assert 'packing_rl' not in sys.modules"
        result=subprocess.run([sys.executable,'-c',code],cwd=Path(__file__).parent,
                              capture_output=True,text=True,timeout=60)
        self.assertEqual(result.returncode,0,result.stderr)
    def test_names_are_identifiers_and_equal_size_boxes_remain_distinct(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'input.json'
            path.write_text(json.dumps([dict(name='new_A',size_cm=[55,35,20]),
                                       dict(name='new_B',size_m=[.55,.35,.2])]))
            result=load_inventory(path)
        self.assertEqual(result['box_labels'],['new_A','new_B'])
        self.assertEqual(result['boxes'][0],result['boxes'][1])

    def test_modified_or_unvalidated_route_cannot_be_loaded_as_validated(self):
        data=dict(profile=PROFILE_TYPE,cycles=[dict(place_target=dict(route_source='offline_frozen_v1',position_m=[1,2,3]))])
        with self.assertRaises(ValueError):require_validation(data)
        data['offline_validation']=validation_stamp(data,'unit fixture')
        require_validation(data)
        changed=copy.deepcopy(data);changed['cycles'][0]['place_target']['position_m'][0]=2
        with self.assertRaises(ValueError):require_validation(changed)
        changed=copy.deepcopy(data);changed['motion_safety']={'calibration':'changed'}
        with self.assertRaises(ValueError):require_validation(changed)

    def test_units_and_duplicate_names_are_not_guessed(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'input.json'
            for rows in ([dict(name='a',size_cm=[0,3,4])],
                         [dict(name='a',size_cm=[1,2,3],size_m=[1,2,3])],
                         [dict(name='a',size_cm=[1,2,3])]*2):
                path.write_text(json.dumps(rows))
                with self.assertRaises(ValueError):load_inventory(path)


if __name__=='__main__':unittest.main()
