import copy
import math
from pathlib import Path
import unittest

import yaml
from luggage_description.doorway_clearance import DoorwayGeometry,rotation_matrix,rotate


class DoorwayClearanceTests(unittest.TestCase):
    def setUp(self):
        self.root=Path(__file__).resolve().parents[3]
        self.scene=yaml.safe_load((self.root/'deployment_ws/config/scene_tf.loading_site.yaml').read_text())
        self.door=DoorwayGeometry.from_scene(self.scene)

    def test_frame_inflation_preserves_five_centimetres_on_each_side(self):
        guards={b['id'].rsplit('_',1)[-1]:b for b in self.door.local_guard_boxes()}
        x,y0,y1,z0,z1=self.door.bounds
        self.assertAlmostEqual(guards['left']['xyz'][1]-guards['left']['size'][1]/2,y1-.05)
        self.assertAlmostEqual(guards['right']['xyz'][1]+guards['right']['size'][1]/2,y0+.05)
        self.assertAlmostEqual(guards['top']['xyz'][2]-guards['top']['size'][2]/2,z1-.05)
        self.assertAlmostEqual(guards['bottom']['xyz'][2]+guards['bottom']['size'][2]/2,z0+.05)
        self.assertLess(guards['left']['xyz'][0]-guards['left']['size'][0]/2,-.785)
        self.assertGreater(guards['left']['xyz'][0]+guards['left']['size'][0]/2,x)

    def test_original_one_centimetre_goal_is_aligned_inside_protected_aperture(self):
        original=(-.765,.398-.185-.01,1.2)
        aligned=self.door.align_box(original,(0,0,0,1),(.59,.37,.24),(0,0,0),(0,0,0,1))
        self.assertLess(aligned[1],original[1])
        self.assertAlmostEqual(.398-(aligned[1]+.185),.056)

    def test_rotation_uses_box_extent_not_only_center(self):
        yaw=math.pi/4
        q=(0,0,math.sin(yaw/2),math.cos(yaw/2))
        dimensions=(.67,.43,.26)
        aligned=self.door.align_box((-.765,.2,1.1),q,dimensions,(0,0,0),(0,0,0,1))
        extent=(dimensions[0]+dimensions[1])/2/math.sqrt(2)
        self.assertGreaterEqual(.398-aligned[1]-extent,.055999)

    def test_inside_floor_target_is_not_raised_or_relocated(self):
        target=(-.145,.295,.65)
        result=self.door.align_box(target,(0,0,0,1),(.59,.37,.24),(0,0,0),(0,0,0,1))
        self.assertEqual(result,target)

    def test_world_transform_keeps_full_tilt(self):
        q=(0,0,math.sin(.7/2),math.cos(.7/2))
        local=self.door.local_guard_boxes()[0]
        world=self.door.world_guard_boxes((1,2,3),q)[0]
        expected=rotate(rotation_matrix(q),local['xyz'])
        self.assertEqual(world['size'],local['size'])
        self.assertTrue(all(abs(world['xyz'][i]-expected[i]-(i+1))<1e-9 for i in range(3)))

    def test_legacy_scene_keeps_original_collision_model(self):
        scene=copy.deepcopy(self.scene)
        del scene['container']['opening']['clearance_m']
        self.assertEqual(DoorwayGeometry.from_scene(scene).local_guard_boxes(),[])

    def test_live_profile_cannot_reduce_margin_below_five_centimetres(self):
        settings=self.door.settings()
        settings['clearance_m']=.049
        with self.assertRaisesRegex(ValueError,'at least'):
            DoorwayGeometry.from_settings(settings,require_site_margin=True)

    def test_box_that_cannot_fit_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'PAYLOAD_TOO_LARGE'):
            self.door.align_box((-.765,0,1.2),(0,0,0,1),(1,2,1),(0,0,0),(0,0,0,1))


if __name__=='__main__':
    unittest.main()
