"""Sample the full configured payload along every centred entry and reverse.

This verifies box/guard/pedestal/configured-cargo geometry, not robot IK or a
measured movable door leaf. Runtime MoveIt still checks the entire robot.
"""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import yaml

from luggage_description.doorway_clearance import rotation_matrix, rotate
from luggage_planning.carry_entry import CarryConfig, CarryEntry
from luggage_planning.doorway_entry import CenteredDoorwayEntry, extent, xyz
from luggage_planning.doorway_motion import DoorwayMotionGuard
from luggage_planning.hardware_loading_cycle import parse_profile
from luggage_planning.placement_approach import lower_final_approach
from luggage_planning.placement_payload import release_above_support
from luggage_planning.policy_place import connect_place_from_pose
from luggage_planning.pose import Pose, Point, Quaternion, MotionSegment
from luggage_planning.suction_candidate_waypoints import quat_mul

ROOT = Path(__file__).resolve().parents[3]


def scene_pose():
    transform = yaml.safe_load((ROOT/'deployment_ws/config/scene_tf.loading_site.yaml').read_text())['static_transforms'][0]
    q = Quaternion()
    for axis, angle in zip(('x','y','z'), transform['rotation_rpy']):
        r = Quaternion(0,0,0,math.cos(angle/2))
        setattr(r, axis, math.sin(angle/2))
        q = quat_mul(r,q)
    return Pose(Point(*transform['translation']),q)


def overlaps(pose, size, obstacle_pose, obstacle_size):
    # Production separating-axis kernel independently checks the route generator.
    checker = CarryEntry.__new__(CarryEntry)
    checker.size = size
    checker.config = NS(pedestal=obstacle_pose, size=obstacle_size, margin=0.)
    return not checker.pedestal_clear(pose)


def samples(poses):
    yield poses[0]
    for start, goal in zip(poses,poses[1:]):
        segment = MotionSegment(type='cartesian', target_pose=goal)
        yield from (s.target_pose for s in connect_place_from_pose(start,[segment],.01,math.radians(2)))


class CenteredEntryTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT/'deployment_ws/config/loading_cycle.site.template.json').read_text())
        _, self.cycles = parse_profile(self.profile)
        self.container = scene_pose()
        self.guard = DoorwayMotionGuard(self.profile)
        self.config = CarryConfig(self.profile['carry_entry'])
        self.entry = CenteredDoorwayEntry(self.guard.geometry,self.config)

    def route(self,index):
        original = self.cycles[index][0]
        target = lower_final_approach(release_above_support(original))
        target,_ = self.guard.align_target(target,original.reference_box_size_m,self.container)
        return self.entry.build(target,original.reference_box_size_m,self.container,
                                [t for t,_ in self.cycles[:index]])

    def test_all_eight_boxes_and_reverse_clear_guards_pedestal_and_previous_configured_boxes(self):
        guards = [(Pose(Point(*g['xyz']),Quaternion(*g['quat'])),g['size'],g['id'])
                  for g in self.guard.collision_boxes(self.container)]
        pedestal = (self.config.pedestal,tuple(v+2*self.config.margin for v in self.config.size),'pedestal')
        for index,(original,_) in enumerate(self.cycles):
            with self.subTest(box=index+1):
                target,report = self.route(index)
                self.assertEqual(target.box_pose,release_above_support(original).box_pose)
                self.assertGreaterEqual(len(target.via),30)
                cargo = [(t.box_pose,t.reference_box_size_m,'previous_%d'%i)
                         for i,(t,_) in enumerate(self.cycles[:index])]
                # Stop at pre-place; do not confuse intended support contact
                # on the final drop with a blocked carry corridor.
                pre = replace(target.box_pose,position=Point(target.box_pose.position.x,
                    target.box_pose.position.y,target.box_pose.position.z+target.clearance_m))
                poses = [*target.via,pre]
                for pose in samples(poses):
                    for obstacle,size,identity in [*guards,pedestal,*cargo]:
                        self.assertFalse(overlaps(pose,target.reference_box_size_m,obstacle,size),
                            'box %d at %s vs %s'%(index+1,xyz(pose),identity))
                # The return uses exactly the reversed successful waypoints;
                # testing with a held box is stricter than normal empty return.
                for pose in samples(list(reversed(poses))):
                    for obstacle,size,identity in guards:
                        self.assertFalse(overlaps(pose,target.reference_box_size_m,obstacle,size),identity)

    def test_crossing_is_on_aperture_centreline_and_lateral_turn_is_fully_inside(self):
        inv = tuple(zip(*rotation_matrix(tuple(asdict(self.container.orientation).values()))))
        def local(p):return rotate(inv,tuple(v-o for v,o in zip(xyz(p),xyz(self.container))))
        for index in range(8):
            target,report = self.route(index)
            outside = Pose(Point(**report['outside_box_pose']['position']),
                           Quaternion(**report['outside_box_pose']['orientation']))
            inside = Pose(Point(**report['inside_box_pose']['position']),
                          Quaternion(**report['inside_box_pose']['orientation']))
            a,b = local(outside),local(inside)
            self.assertLessEqual(abs(a[1]+.265),.100000001)
            self.assertAlmostEqual(a[1],report['centre_local_y_m'])
            self.assertAlmostEqual(b[1],a[1])
            self.assertAlmostEqual(b[2],a[2])
            self.assertGreaterEqual(b[0]-extent(inside,target.reference_box_size_m,inv)[0],
                                    self.guard.geometry.bounds[0]+.15-1e-9)

    def test_taught_p2_entry_never_reverses_outward_for_any_box(self):
        inv = tuple(zip(*rotation_matrix(tuple(asdict(self.container.orientation).values()))))
        def longitudinal(p):
            point,origin=xyz(p),xyz(self.container)
            # Horizontal advance toward the doorway; container pitch makes a
            # pure vertical descent look like a 1cm reverse in 3-D local X.
            return sum(inv[0][axis]*(point[axis]-origin[axis]) for axis in (0,1))
        for index,(original,_) in enumerate(self.cycles):
            with self.subTest(box=index+1):
                target = lower_final_approach(release_above_support(original))
                target,_ = self.guard.align_target(target,original.reference_box_size_m,self.container)
                start = Pose(Point(-.55,.15,1.1), target.box_pose.orientation)
                route,report = self.entry.build(target,original.reference_box_size_m,self.container,
                    [t for t,_ in self.cycles[:index]],start_box_pose=start)
                self.assertEqual(report['start_source'],'measured_p2')
                self.assertEqual(route.via[0],start)
                progress=[longitudinal(p) for p in route.via]
                self.assertTrue(all(b >= a-1e-8 for a,b in zip(progress,progress[1:])),progress[:5])
                if index == 0:
                    self.assertAlmostEqual(report['inside_drop_m'], .12, places=3)
                    self.assertTrue(any(abs(p.position.x-route.via[-1].position.x)>.1
                                        and p.position.z < report['inside_box_pose']['position']['z']-.1
                                        for p in route.via))
                elif index == 2:
                    self.assertAlmostEqual(route.clearance_m, .075)
                    self.assertLess(route.via[-1].position.z, 1.19)
                elif index == 3:
                    self.assertEqual(report['door_lower_m'], 0.)
                    self.assertEqual(report['entry_mode'],'target_aisle')
                    self.assertEqual(report['inside_drop_m'], 0.)
                elif index == 5:
                    self.assertAlmostEqual(route.clearance_m, .075)
                    self.assertEqual(route.box_pose, target.box_pose)
                    self.assertEqual(report['inside_drop_m'], 0.)
                else:
                    self.assertEqual(report['inside_drop_m'], 0.)

    def test_impossible_width_or_no_inside_pose_fails_closed(self):
        target = self.cycles[5][0]
        with self.assertRaisesRegex(ValueError,'too wide'):
            self.entry.build(target,(.2,2.,.2),self.container)
        outside = replace(target,box_pose=Pose(Point(0,1,1)))
        with self.assertRaisesRegex(ValueError,'fully-inside'):
            self.entry.build(outside,target.reference_box_size_m,self.container)

    def test_all_measured_entries_clear_configured_obstacles(self):
        guards = [(Pose(Point(*g['xyz']),Quaternion(*g['quat'])),g['size'],g['id'])
                  for g in self.guard.collision_boxes(self.container)]
        pedestal = (self.config.pedestal,
                    tuple(v+2*self.config.margin for v in self.config.size),'pedestal')
        for index in range(8):
            with self.subTest(box=index+1):
                original = self.cycles[index][0]
                target = lower_final_approach(release_above_support(original))
                target,_ = self.guard.align_target(target,original.reference_box_size_m,self.container)
                start = Pose(Point(-.55,.15,1.1),target.box_pose.orientation)
                route,_ = self.entry.build(target,original.reference_box_size_m,self.container,
                    [t for t,_ in self.cycles[:index]],start_box_pose=start)
                if index==3:
                    # Right-side box must not first align toward the already
                    # loaded blue stack on the opposite side.
                    self.assertLessEqual(route.via[1].position.y,start.position.y)
                    self.assertEqual(route.box_pose,target.box_pose)
                pre = replace(route.box_pose,position=Point(route.box_pose.position.x,
                    route.box_pose.position.y,route.box_pose.position.z+route.clearance_m))
                cargo = [(t.box_pose,t.reference_box_size_m,'previous_%d'%i)
                         for i,(t,_) in enumerate(self.cycles[:index])]
                for pose in samples([*route.via[1:],pre]):
                    for obstacle,size,identity in [*guards,pedestal,*cargo]:
                        self.assertFalse(overlaps(pose,route.reference_box_size_m,obstacle,size),
                            'box %d at %s vs %s'%(index+1,xyz(pose),identity))

    def test_console_applies_entry_to_every_box_and_records_it_for_reverse_return(self):
        from test_hardware_loading_ports import load_console
        from luggage_planning.pick_return_path import PickReturnPath
        module = load_console()
        for index,(target,_) in enumerate(self.cycles):
            with self.subTest(box=index+1):
                driver = module.CommissioningDriver()
                size = target.reference_box_size_m
                actual = Pose(Point(0,1.6,1.3),Quaternion(1,0,0,0))
                driver._args = NS(place_target_data=target)
                driver._measured_grasp = ((0,0,size[2]/2),Quaternion(1,0,0,0))
                driver._prior_place_targets = tuple(t for t,_ in self.cycles[:index])
                driver.doorway_guard,driver.doorway_entry = self.guard,self.entry
                driver.carry_entry_config = self.config
                driver.lookup_pose = lambda parent,child,**kw:self.container if child=='container_link' else actual
                messages=[]
                driver.get_logger = lambda:NS(info=messages.append)
                segments = driver.prepare_place_segments([],NS(frame='world',yaw_valid=True,box_size=size),NS(frame='world'))
                report = json.loads(next(m.split(': ',1)[1] for m in messages
                                         if m.startswith('centred doorway entry: ')))
                self.assertEqual(driver._doorway_entry_strategy,self.entry.strategy)
                pre = next(s for s in segments if s.name=='pre_place')
                self.assertGreater(len(pre.waypoints),30)
                self.assertEqual(segments[-2].name,'place_descend')
                self.assertEqual(segments[-1].name,'place_retreat')
                if report['entry_mode']=='centre':
                    self.assertLessEqual(abs(report['centre_local_y_m']+.265),.100000001)
                else:
                    self.assertGreater(report['centre_local_y_m'],-.265)
                driver._pick_return_path = PickReturnPath()
                driver._pick_return_path.poses = [actual]
                driver.lookup_pose = lambda *args,**kw:pre.target_pose
                driver.record_completed_motion(pre)
                reverse = driver._pick_return_path.continuous_segment()
                self.assertEqual(reverse.target_pose,actual)
                self.assertEqual(reverse.waypoints,list(reversed([*pre.waypoints,pre.target_pose])))

    def test_transit_lowers_about_eight_cm_without_flying_over_irrelevant_tall_stack(self):
        lowerings=[]
        for index in range(8):
            _,report=self.route(index)
            if index<3:
                self.assertEqual(report['centre_offset_m'],0.)
            else:
                # The slot's own pre-place height limits lowering regardless
                # of its sequence index (side stacks now precede the middle).
                self.assertGreaterEqual(report['lowered_from_all_cargo_baseline_m'],0.)
                self.assertLess(report['lowered_from_all_cargo_baseline_m'],.085)
                self.assertLessEqual(abs(report['centre_offset_m']),.10)
                self.assertTrue(all(1<=number<=index for number in report['height_relevant_prior_boxes']))
                lowerings.append(report['lowered_from_all_cargo_baseline_m'])
        self.assertGreater(max(lowerings),.075)
