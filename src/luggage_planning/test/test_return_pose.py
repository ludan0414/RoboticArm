import math
import unittest

from luggage_planning.pose import Point, Pose, Quaternion
from luggage_planning.return_pose import adjusted_return_pose
from luggage_planning.suction_candidate_waypoints import quat_mul, quat_conjugate, quat_rotate


class ReturnPoseTest(unittest.TestCase):
    def test_offset_is_in_actual_base_axes_and_camera_views_base_minus_x(self):
        nominal = Pose(Point(.2, 1.6, .86), Quaternion())
        camera = Pose(Point(), Quaternion(0, math.sqrt(.5), 0, math.sqrt(.5)))
        for base_q in (Quaternion(), Quaternion(0, 0, -math.sqrt(.5), math.sqrt(.5)),
                       quat_mul(Quaternion(0, 0, math.sin(.3), math.cos(.3)),
                                Quaternion(math.sin(.1), 0, 0, math.cos(.1)))):
            base = Pose(Point(.4, -.2, .86), base_q)
            result = adjusted_return_pose(nominal, base, camera)
            delta = tuple(getattr(result.position,k)-getattr(nominal.position,k) for k in ('x','y','z'))
            local = quat_rotate(quat_conjugate(base_q), delta)
            for actual, expected in zip(local, (.8,0,-.2)):
                self.assertAlmostEqual(actual, expected, places=10)
            tool_q_base = quat_mul(quat_conjugate(base_q), result.orientation)
            optical = quat_rotate(quat_mul(tool_q_base, camera.orientation), (0,0,1))
            image_down = quat_rotate(quat_mul(tool_q_base, camera.orientation), (0,1,0))
            for actual, expected in zip(optical,(-1,0,0)):
                self.assertAlmostEqual(actual,expected,places=10)
            for actual, expected in zip(image_down,(0,0,-1)):
                self.assertAlmostEqual(actual,expected,places=10)

    def test_tilted_camera_mount_points_full_optical_axis_along_negative_x(self):
        camera = Pose(Point(), quat_mul(Quaternion(0,math.sin(.5),0,math.cos(.5)),
                                       Quaternion(0,0,math.sin(.3),math.cos(.3))))
        result = adjusted_return_pose(Pose(), Pose(), camera)
        view = quat_rotate(quat_mul(result.orientation,camera.orientation), (0,0,1))
        for actual, expected in zip(view,(-1,0,0)):
            self.assertAlmostEqual(actual,expected,places=10)

    def test_invalid_mount_quaternion_and_nonfinite_offset_refused(self):
        with self.assertRaises(ValueError):
            adjusted_return_pose(Pose(), Pose(), Pose(Point(),Quaternion(0,0,0,0)))
        camera = Pose(Point(), Quaternion(0,math.sqrt(.5),0,math.sqrt(.5)))
        with self.assertRaises(ValueError):
            adjusted_return_pose(Pose(), Pose(), camera, [float('nan'),0,0])
