import unittest
from reach_projection import furthest_reachable


class ReachProjectionTests(unittest.TestCase):
    def test_extends_to_workspace_boundary_and_reports_unreached_distance(self):
        def two_link_ik(point):
            # Unit links are straight at r=2; the inner forbidden region
            # demonstrates that reachability is not monotonic from the origin.
            radius = sum(v*v for v in point)**.5
            return point if .8 <= radius <= 2 else None
        answer = furthest_reachable((0,0,0), (3,0,0), two_link_ik)
        self.assertAlmostEqual(answer['point'][0], 2., delta=.001)
        self.assertAlmostEqual(answer['shortfall_m'], 1., delta=.001)

    def test_never_accepts_an_unvalidated_projected_point(self):
        self.assertIsNone(furthest_reachable((0,0,0), (3,0,0), lambda p: None))

    def test_far_target_still_finds_a_small_reachable_annulus(self):
        answer=furthest_reachable((0,0,0),(10,0,0),
            lambda point:point if .2<=point[0]<=1.5 else None)
        self.assertIsNotNone(answer)
        self.assertAlmostEqual(answer['point'][0],1.5,delta=.001)
        self.assertAlmostEqual(answer['shortfall_m'],8.5,delta=.001)

    def test_reachable_original_goal_is_unchanged(self):
        answer = furthest_reachable((0,0,0), (1,0,0), lambda p: p)
        self.assertEqual(answer['point'], (1.,0.,0.))
        self.assertEqual(answer['shortfall_m'], 0.)


if __name__ == '__main__':
    unittest.main()
