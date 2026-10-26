"""Behavioral doorway goals and real scene adapter with fake ROS transport."""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import yaml
from luggage_description.doorway_clearance import DoorwayGeometry
from luggage_planning.doorway_motion import DoorwayMotionGuard
from luggage_planning.pose import Point,Pose,Quaternion
from luggage_planning.policy_place import PlaceTarget
from luggage_planning.suction_candidate_waypoints import quat_mul,quat_rotate


ROOT=Path(__file__).resolve().parents[3]


def pose_message(xyz=(0,0,0),quat=(0,0,0,1)):
    return SimpleNamespace(position=SimpleNamespace(x=xyz[0],y=xyz[1],z=xyz[2]),
                           orientation=SimpleNamespace(x=quat[0],y=quat[1],z=quat[2],w=quat[3]))


def client_type():
    # Execute the production adapter method against a fake service boundary;
    # no ROS installation or robot/controller is needed on the test host.
    path=ROOT/'src/luggage_planning/luggage_planning/planning_scene_client.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    owner=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='PlanningSceneClient')
    owner.body=[node for node in owner.body if isinstance(node,ast.FunctionDef) and node.name=='ensure_doorway_clearance']
    helpers=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ('_xyz_quat_from_pose','acm_pair_allowed')]
    scope=dict(math=math,Quaternion=Quaternion,quat_mul=quat_mul,quat_rotate=quat_rotate,
        BOX_OBJECT_ID='pickup_box',DEFAULT_ATTACH_LINK='suction_contact_frame',
        SolidPrimitive=SimpleNamespace(BOX=1),PlanningSceneComponents=SimpleNamespace(
            WORLD_OBJECT_GEOMETRY=16,ROBOT_STATE_ATTACHED_OBJECTS=4,ALLOWED_COLLISION_MATRIX=128))
    exec(compile(ast.Module(body=helpers+[owner],type_ignores=[]),str(path),'exec'),scope)
    return scope['PlanningSceneClient']


class DoorwayMotionTests(unittest.TestCase):
    def setUp(self):
        scene=yaml.safe_load((ROOT/'deployment_ws/config/scene_tf.loading_site.yaml').read_text())
        self.geometry=DoorwayGeometry.from_scene(scene)
        self.guard=DoorwayMotionGuard({'doorway_clearance':self.geometry.settings()})
        self.boxes=self.guard.collision_boxes(Pose())

    def test_missing_profile_margin_is_not_silently_disabled(self):
        with self.assertRaisesRegex(ValueError,'regenerate'):
            DoorwayMotionGuard({})

    def test_only_vias_are_shifted_and_final_slot_stays_unchanged(self):
        final=Pose(Point(-.145,.295,.65),Quaternion())
        unsafe=Pose(Point(-.765,.398-.185-.01,1.2),Quaternion())
        target=PlaceTarget(final,.15,(unsafe,),(.59,.37,.24))
        aligned,report=self.guard.align_target(target,(.59,.37,.24),Pose())
        self.assertEqual(aligned.box_pose,final)
        self.assertEqual(aligned.via[0].orientation,unsafe.orientation)
        self.assertEqual(report['adjusted_via_poses'],1)
        self.assertAlmostEqual(.398-aligned.via[0].position.y-.185,.056)

    def test_unsafe_final_pose_is_rejected_instead_of_moving_the_placement(self):
        target=PlaceTarget(Pose(Point(-.765,.3,1.2),Quaternion()),.15)
        with self.assertRaisesRegex(ValueError,'PLACE_TARGET_UNSAFE'):
            self.guard.align_target(target,(.59,.37,.24),Pose())

    def scene_client(self):
        client=client_type()()
        self.events=[]
        objects=[]
        for box in self.boxes:
            objects.append(SimpleNamespace(id=box['id'],header=SimpleNamespace(frame_id='world'),
                pose=pose_message(),primitives=[SimpleNamespace(type=1,dimensions=list(box['size']))],
                primitive_poses=[pose_message(box['xyz'],box['quat'])]))
        names=[box['id'] for box in self.boxes]+['pickup_box']
        matrix=SimpleNamespace(entry_names=names,entry_values=[SimpleNamespace(enabled=[False]*5) for _ in names])
        payload=SimpleNamespace(link_name='suction_contact_frame',object=SimpleNamespace(id='pickup_box',
            primitives=[SimpleNamespace(type=1,dimensions=[.59,.37,.24])]))
        self.scene=SimpleNamespace(world=SimpleNamespace(collision_objects=objects),
            allowed_collision_matrix=matrix,robot_state=SimpleNamespace(attached_collision_objects=[payload]))
        client.add_collision_boxes=lambda boxes,**kwargs:(self.events.append('apply') or True,'applied')
        client.set_acm_pairs=lambda pairs,allowed,**kwargs:(self.events.append(('acm',pairs,allowed)) or True,'applied')
        client.get_scene=lambda *args,**kwargs:self.scene
        return client

    def test_geometry_is_read_back_without_acm_queries_or_writes(self):
        client=self.scene_client()
        requested=[]
        client.set_acm_pairs=lambda *args,**kwargs:self.fail('doorway must not rewrite ACM')
        client.get_scene=lambda components,**kwargs:(requested.append(components) or self.scene)
        del self.scene.allowed_collision_matrix
        self.assertTrue(client.ensure_doorway_clearance(self.boxes,True,[.59,.37,.24])[0])
        self.assertEqual(self.events,['apply'])
        self.assertEqual(requested,[16|4])

    def test_apply_failure_stops_before_readback(self):
        client=self.scene_client()
        client.add_collision_boxes=lambda *args,**kwargs:(False,'unavailable')
        client.get_scene=lambda *args,**kwargs:self.fail('must not continue after apply failure')
        self.assertFalse(client.ensure_doorway_clearance(self.boxes)[0])

    def test_missing_guard_or_wrong_dimensions_is_rejected(self):
        client=self.scene_client()
        self.scene.world.collision_objects.pop()
        self.assertFalse(client.ensure_doorway_clearance(self.boxes)[0])
        client=self.scene_client()
        self.scene.world.collision_objects[0].primitives[0].dimensions=[.001]*3
        self.assertFalse(client.ensure_doorway_clearance(self.boxes)[0])

    def test_missing_attached_payload_or_wrong_payload_dimensions_are_rejected(self):
        client=self.scene_client()
        self.scene.robot_state.attached_collision_objects=[]
        self.assertFalse(client.ensure_doorway_clearance(self.boxes,True,[.59,.37,.24])[0])
        client=self.scene_client()
        self.scene.robot_state.attached_collision_objects[0].object.primitives[0].dimensions=[.1,.1,.1]
        self.assertFalse(client.ensure_doorway_clearance(self.boxes,True,[.59,.37,.24])[0])

    def test_nonfinite_scene_geometry_is_rejected(self):
        client=self.scene_client()
        self.scene.world.collision_objects[0].primitives[0].dimensions[0]=float('nan')
        self.assertFalse(client.ensure_doorway_clearance(self.boxes)[0])

    def test_empty_tool_does_not_gate_on_collision_permissions(self):
        client=self.scene_client()
        self.scene.allowed_collision_matrix.entry_values[4].enabled[0]=True
        self.assertTrue(client.ensure_doorway_clearance(self.boxes)[0])

    def test_moveit_object_pose_representation_is_accepted(self):
        client=self.scene_client()
        for obj in self.scene.world.collision_objects:
            obj.pose=obj.primitive_poses[0]
            obj.primitive_poses=[pose_message()]
        self.assertTrue(client.ensure_doorway_clearance(self.boxes)[0])

    def test_empty_tool_does_not_require_pickup_box_in_acm(self):
        client=self.scene_client()
        self.scene.allowed_collision_matrix.entry_names=['elfin_base_link']
        self.scene.allowed_collision_matrix.entry_values=[
            SimpleNamespace(enabled=[False])]
        ok,message=client.ensure_doorway_clearance(self.boxes)
        self.assertTrue(ok,message)
        self.assertEqual(self.events,['apply'])

    def test_carrying_checks_payload_geometry_without_requiring_acm(self):
        client=self.scene_client()
        names=[box['id'] for box in self.boxes]
        self.scene.allowed_collision_matrix.entry_names=names
        self.scene.allowed_collision_matrix.entry_values=[
            SimpleNamespace(enabled=[False]*len(names)) for _ in names]
        ok,message=client.ensure_doorway_clearance(self.boxes,True,[.59,.37,.24])
        self.assertTrue(ok,message)
        self.assertEqual(self.events,['apply'])


if __name__=='__main__':
    unittest.main()
