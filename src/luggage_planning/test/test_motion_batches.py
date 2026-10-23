import unittest

from luggage_planning.motion_batches import batch_cartesian_segments
from luggage_planning.pick_return_path import PickReturnPath
from luggage_planning.pose import MotionSegment, Point, Pose


def pose(x):
    return Pose(Point(x,0,1))


def flatten(segments):
    return [p for segment in segments for p in [*segment.waypoints,segment.target_pose]]


class MotionBatchTests(unittest.TestCase):
    def test_eight_stops_retain_every_via_in_order_and_disable_shortcuts(self):
        original=[MotionSegment(name='place_via_%d'%i,type='cartesian',
            target_pose=pose(2*i+1),waypoints=[pose(2*i)],allow_ompl_fallback=True)
            for i in range(53)]
        batches=batch_cartesian_segments(original,8)
        self.assertEqual(len(batches),8)
        self.assertEqual(flatten(batches),flatten(original))
        self.assertEqual(batches[-1].name,original[-1].name)
        self.assertTrue(all(s.required_cartesian_fraction==1 and not s.allow_ompl_fallback
                            for s in batches))

    def test_return_has_seven_stops_plus_final_observe_without_losing_route(self):
        path=PickReturnPath()
        path.record('pick_retreat',pose(0))
        path.record('place_via_50',pose(53),[pose(i) for i in range(1,53)])
        segments=[]
        while path.poses:
            segment=path.next_segment()
            self.assertEqual(path.next_segment(),segment)
            segments.append(segment)
            path.settled()
        self.assertEqual(len(segments),7)
        self.assertEqual([p.position.x for p in flatten(segments)],list(range(53,-1,-1)))
        self.assertEqual(segments[-1].target_pose,pose(0))

    def test_single_segment_route_is_kept_and_empty_route_needs_no_action(self):
        segment=MotionSegment(type='cartesian',target_pose=pose(1))
        self.assertEqual(batch_cartesian_segments([],8),[])
        self.assertIs(batch_cartesian_segments([segment],8)[0],segment)

    def test_rejects_merging_pose_targets_that_cannot_honor_vias(self):
        with self.assertRaisesRegex(ValueError,'Cartesian'):
            batch_cartesian_segments([MotionSegment(type='pose_target')]*9,8)

    def test_ground_turns_remain_full_stops_in_both_directions(self):
        names=['ground_entry_outer_lift','ground_entry_outside_retract','ground_entry_high_lift']
        original=[MotionSegment(name=(names[i] if i<3 else 'place_via_%d'%i),
                    type='cartesian',target_pose=pose(i+1)) for i in range(53)]
        batches=batch_cartesian_segments(original,8)
        self.assertTrue(set(names)<=set(s.name for s in batches))
        path=PickReturnPath()
        path.record('pick_retreat',pose(0))
        for s in batches:
            path.record(s.name,s.target_pose,s.waypoints)
        endpoints=[]
        while path.poses:
            endpoints.append(path.next_segment().target_pose.position.x)
            path.settled()
        self.assertEqual(len(endpoints),7)
        self.assertTrue({0,1,2,3}<=set(endpoints))
