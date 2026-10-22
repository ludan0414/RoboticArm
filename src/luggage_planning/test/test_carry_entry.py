import math
from types import SimpleNamespace
import unittest
import json
from pathlib import Path
from luggage_planning.carry_entry import CarryConfig, CarryEntry
from luggage_planning.pose import Pose, Point, Quaternion, MotionSegment


def fixture(x=0, y=1.6, offcenter=0):
    config = CarryConfig(dict(strategy='outside_retract_high_lift_v1',
        pedestal_center_world_pose=dict(position_m=[0,0,.43],
            orientation_xyzw=[0,0,math.sin(-math.pi/4),math.cos(-math.pi/4)]),
        pedestal_size_m=[1.05,.74,.86], pedestal_margin_m=.05,
        high_bottom_above_pedestal_m=.15, outer_lift_tool_world_z_m=.86,
        staging_radius_m=1.25))
    detection = SimpleNamespace(frame='world', yaw_valid=True, detection_yaw=0,
        box_xyz=(x,y,.1), box_quat=(0,0,0,1), box_size=(.55,.35,.2))
    candidate = SimpleNamespace(frame='world', contact=Pose(Point(x+offcenter,y,.2),Quaternion()))
    entry = CarryEntry(config,detection,candidate)
    actual = Pose(Point(x+offcenter,y,.32),Quaternion(1,0,0,0))
    first = MotionSegment(name='place_via_0',type='cartesian',
        target_pose=Pose(Point(-.4,.9,1.22),actual.orientation),
        required_cartesian_fraction=1, allow_ompl_fallback=False)
    return entry,actual,first


class CarryEntryTests(unittest.TestCase):
    def test_exported_deployment_route_connects_to_original_pick_and_keeps_clearance(self):
        from luggage_planning.policy_place import parse_place_target, build_place_segments
        from luggage_planning.suction_candidate_waypoints import build_candidate_pick_segments
        from luggage_planning.hardware_motion_guard import HardwareMotionGuard
        root=Path(__file__).resolve().parents[3]
        profile=json.loads((root/'deployment_ws/config/loading_cycle.site.template.json').read_text())
        yaw=-3.131557004068548-math.pi/2
        detection=SimpleNamespace(frame='world',yaw_valid=True,detection_yaw=yaw,
            box_xyz=(0,1.6,.1),box_quat=(0,0,math.sin(yaw/2),math.cos(yaw/2)),box_size=(.55,.35,.2))
        candidate=SimpleNamespace(frame='world',contact=Pose(Point(0,1.6,.2),Quaternion()))
        actual=build_candidate_pick_segments(candidate,detection_yaw=yaw,yaw_valid=True)[3].target_pose
        segments=build_place_segments(parse_place_target(profile['cycles'][0]['place_target']),detection,candidate)
        entry=CarryEntry(CarryConfig(profile['carry_entry']),detection,candidate)
        guard=HardwareMotionGuard(profile)
        route=entry.connect(actual,segments)
        previous=actual
        for segment in route[:-1]:
            self.assertTrue(guard.check_segment(previous,segment.target_pose,segment.name)[0],segment.name)
            self.assertTrue(entry.check_segment(previous,segment)[0],segment.name)
            previous=segment.target_pose

    def test_lift_then_outside_retract_then_high_lift_before_transfer(self):
        entry,actual,first = fixture()
        route = entry.connect(actual,[first])
        outer = next(s for s in route if s.name=='ground_entry_outer_lift')
        retract = next(s for s in route if s.name=='ground_entry_outside_retract')
        high = next(s for s in route if s.name=='ground_entry_high_lift')
        self.assertLess(route.index(outer),route.index(retract))
        self.assertLess(route.index(retract),route.index(high))
        self.assertAlmostEqual(outer.target_pose.position.y,1.6)
        self.assertAlmostEqual(retract.target_pose.position.y,1.25)
        self.assertGreaterEqual(entry.bottom(entry.box_pose(high.target_pose)),1.01)
        previous=actual
        for segment in route:
            self.assertEqual(segment.type,'cartesian')
            self.assertEqual(segment.required_cartesian_fraction,1)
            self.assertFalse(segment.allow_ompl_fallback)
            self.assertTrue(entry.check_segment(previous,segment)[0],segment.name)
            previous=segment.target_pose
        self.assertIs(route[-1],first)

    def test_variable_pick_and_off_center_grasp_use_measured_box(self):
        entry,actual,first=fixture(.1,1.443,.06)
        box=entry.box_pose(actual)
        self.assertAlmostEqual(box.position.x,.1)
        self.assertAlmostEqual(box.position.y,1.443)
        self.assertAlmostEqual(box.position.z,.22)
        self.assertTrue(entry.connect(actual,[first]))

    def test_low_crossing_rejected_with_margin_before_contact(self):
        entry,actual,_=fixture()
        actual.position.z=.5
        blocked=MotionSegment(name='ground_entry_outside_retract',type='cartesian',
            target_pose=Pose(Point(0,0,.5),actual.orientation))
        self.assertFalse(entry.check_segment(actual,blocked)[0])

    def test_inside_pedestal_pick_cannot_create_blind_transfer(self):
        entry,actual,first=fixture(0,.6)
        with self.assertRaises(ValueError):
            entry.connect(actual,[first])

    def test_empty_tool_retreat_not_checked_as_attached_box(self):
        entry,actual,_=fixture()
        segment=MotionSegment(name='place_retreat',type='cartesian',
                              target_pose=Pose(Point(),actual.orientation))
        self.assertTrue(entry.check_segment(actual,segment)[0])


if __name__=='__main__':
    unittest.main()
